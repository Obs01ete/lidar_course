/**
 * MIT License
 *
 * Copyright (c) 2020 Dmitrii Khizbullin <dmitrii.khizbullin@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#pragma once

#include <cassert>
#include <vector>
#include <unordered_map>
#include <cstdint>

#include <Eigen/Geometry>

#include "gpsimu_t.h"
#include "point_cloud_kernels.h"
#include "constrained_planar_cuts.h"
#include "clusters_and_hulls.h"
#include "processor_params.h"
#include "ransac_ground.h"


namespace lidar_course {


// Eigen's classes may be too verbose to declare, so it could be
// a good idea to give them short names with a typedef.
typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> Transform3f;


// Processor is the main processing class, that performs scan aggregation,
// ground plane estimation and subtraction, clustering of points and
// convex hull extraction.
class Processor
{
private:

    // Structure CloudAndMetadata is a group of a historical point cloud and
    // a correspondin transformation of it to the next scan's coordinates.
    struct CloudAndMetadata
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;
        Transform3f transform_to_next;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    // Settings of processing are shared between a visualizer and a processor
    std::shared_ptr<ProcessorParams> m_params;
    // A queue that keeps historical scans for aggregation
    std::deque<CloudAndMetadata> m_queue;

public:
    Processor(const std::shared_ptr<ProcessorParams>& params) :
        m_params(params),
        m_queue()
    {
    }

    // process() performs processing of the provided current LiDAR scan
    // and returns a labeled cloud along with convex hulls of clusters.
    // gpsimu_ptr is an optional parameter that holds GPS/IMU data from
    // OXTS RT3000 device.
    auto process(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr,
        const std::unique_ptr<kitti_parser::gpsimu_t>& gpsimu_ptr)
    {
        // For diagnostic putposes we could peek into the min-max values of
        // points at all three X, Y and Z coordinates.
        if (false)
        {
            // View XYZ min max
            auto minmax = point_cloud_limits<pcl::PointXYZI>(input_cloud_ptr);
            for (int i = 0; i < 3; i++)
            {
                std::cout << minmax[i].first << " " << minmax[i].second << std::endl;
            }
        }

        // For the sake of speed of processing we may subsample the original point
        // cloud since real-time operation for autonomous driving is more critical
        // than the accuracy of the algorithm.
        auto decimated_cloud_ptr = decimate_cloud<pcl::PointXYZI>(
            input_cloud_ptr, m_params->m_decimation_coef);

        // We accumulate the incoming scans along with their localization metadata
        // into a deque to perform subsequent aggregation.
        {
            Transform3f next_veh_pose_vs_curr = Transform3f::Identity();
            if (gpsimu_ptr)
            {
                float frame_interval_sec = 0.1f;

                // First, we need to calculate yaw change given the yaw rate
                // (angular speed over Z axis) and the time inteval between frames.
                float angle_z = gpsimu_ptr->wz * frame_interval_sec;
                auto rot = Eigen::AngleAxisf(angle_z, Eigen::Vector3f::UnitZ());
                next_veh_pose_vs_curr.rotate(rot);

                // Second, we need a translation transform to the next frame
                // given the speed of the ego-vehicle and the frame interval.
                next_veh_pose_vs_curr.translate(Eigen::Vector3f(
                    gpsimu_ptr->vf * frame_interval_sec,
                    gpsimu_ptr->vl * frame_interval_sec,
                    0.0f
                ));
            }

            // Since later we want to aggregate all scans into the coordinate
            // frame of the last scans, we need the inverse transform.
            auto curr_veh_pose_vs_next = next_veh_pose_vs_curr.inverse();

            // Put the resulting pair of the cloud and the transform into a queue.
            auto cloud_and_metadata = CloudAndMetadata{decimated_cloud_ptr, curr_veh_pose_vs_next};
            m_queue.push_back(cloud_and_metadata);
            while (m_queue.size() > m_params->m_num_clouds)
            {
                m_queue.pop_front();
            }
        }

        // We accumulate the transforms starting from the latest back in time and
        // transform each historical point cloud into the coordinates of the current frame.
        auto aggregated_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
        Eigen::Matrix4f aggragated_transform = Eigen::Matrix4f::Identity();
        for (int i = m_queue.size()-1; i >= 0; i--)
        {
            const auto& cloud_and_metadata = m_queue[i];
            const auto& cloud_ptr = cloud_and_metadata.cloud_ptr;
            const auto& trans = cloud_and_metadata.transform_to_next;
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr;
            if (i != m_queue.size()-1)
            {
                aggragated_transform *= trans.matrix();
                transformed_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
                pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, aggragated_transform);
            }
            else
            {
                // For the current scan no need to transform
                transformed_cloud_ptr = cloud_ptr;
            }
            
            // Concatenate the transformed point cloud into the aggregate cloud
            *aggregated_cloud_ptr += *transformed_cloud_ptr;
        }

        // Transform the aggregated cloud to vehicle coordinates, which
        // have their origin on the ground below the LiDAR.
        auto cloud_vehicle_coords_ptr =
            transform_to_vehicle_coords<pcl::PointXYZI>(aggregated_cloud_ptr);

        // For further processing we do not need intensity of points
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*cloud_vehicle_coords_ptr, *xyz_cloud_ptr);

        // Remove ground points based on RANSAC ground plane estimation
        auto cloud_after_ground_ptr = (m_params->m_remove_ground) ?
            remove_ground_ransac(xyz_cloud_ptr) :
            xyz_cloud_ptr;

        // Finally we would like to detect objects on the scene, which is done
        // by means of clusterization and convex hull extraction.
        CloudAndClusterHulls cloud_and_cluster_hulls;
        if (m_params->m_do_clusterize)
        {
            auto cpc_labeled_cloud = constrained_planar_cuts_segmentation<pcl::PointXYZ>(cloud_after_ground_ptr);
            cloud_and_cluster_hulls = find_primary_clusters(cpc_labeled_cloud);
        }
        else
        {
            // If clusterization is disabled, then simply pass along the original cloud
            auto zero_labeled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL> >();
            pcl::copyPointCloud(*cloud_after_ground_ptr, *zero_labeled_cloud);
            cloud_and_cluster_hulls = CloudAndClusterHulls {
                zero_labeled_cloud,
                std::vector<ClusterWithHull>()
            };
        }

        
        return cloud_and_cluster_hulls;
    }

    // In the beginning of a scene (sequence) we need to reset the processor's state
    void reset()
    {
        m_queue.clear();
    }

};


} // namespace lidar_course
