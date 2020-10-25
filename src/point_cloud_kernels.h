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

#include <array>
#include <limits>
#include <random>
#include <functional>
#include <unordered_set>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>

#include <Eigen/Geometry>


namespace lidar_course {


typedef std::array<std::pair<float, float>, 3> MinMax;


// Figure out min and max point coodinates for X, Y and Z axes.
template<typename PointT>
MinMax point_cloud_limits(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr)
{
    // XYZ min max
    const float inf = std::numeric_limits<float>::infinity();
    MinMax minmax = {{{inf, -inf}, {inf, -inf}, {inf, -inf}}};
    for (const auto& point : input_cloud_ptr->points)
    {
        for (int i = 0; i < 3; i++)
        {
            minmax[i].first = std::min(minmax[i].first, point.data[i]);
            minmax[i].second = std::max(minmax[i].second, point.data[i]);
        }
    }
    return minmax;
}


// Downsample the cloud with a regular pattern of picking each Nth point.
template<typename PointT>
auto decimate_cloud(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr,
    size_t decimation_coef)
{
    auto decimaterd_ptr = std::make_shared<typename pcl::PointCloud<PointT>>();
    for (size_t i = 0; i < input_cloud_ptr->size(); i++)
    {
        if (i % decimation_coef == 0)
        {
            const auto& point = input_cloud_ptr->points[i];
            decimaterd_ptr->push_back(point);
        }
    }
    return decimaterd_ptr;
}


// This function assumes that vehicle's coordinate system origin is
// on the ground right below the Velodyne LiDAR mount
template<typename PointT>
auto transform_to_vehicle_coords(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr)
{
    auto scan_vehicle_coords_ptr = std::make_shared<typename pcl::PointCloud<PointT> >();
    Eigen::Matrix4f lidar_to_vehicle = Eigen::Matrix4f::Identity();
    lidar_to_vehicle(2, 3) = 1.73;
    pcl::transformPointCloud(*input_cloud_ptr, *scan_vehicle_coords_ptr, lidar_to_vehicle);
    return scan_vehicle_coords_ptr;
}


// Remove ground by hard condition on Z below ground_threshold
template<typename PointT>
auto remove_ground_hard(
    typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr,
    float ground_threshold = 0.2f)
{
    typename pcl::PointCloud<PointT>::Ptr scan_no_ground_ptr(new typename pcl::PointCloud<PointT>());
    typename pcl::ConditionAnd<PointT>::Ptr range_cond(new typename pcl::ConditionAnd<PointT>());
    range_cond->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(new
        typename pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, ground_threshold)));
    typename pcl::ConditionalRemoval<PointT> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(input_cloud_ptr);
    condrem.setKeepOrganized(true);
    condrem.filter(*scan_no_ground_ptr);
    return scan_no_ground_ptr;
}


} // namespace lidar_course
