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

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include "interface_types.h"


namespace {
    // A special label that is assigned
    // to points that do not belong to any cluster.
    constexpr std::uint32_t UNCLUSTERED_LABEL = 0;
    // Due to existance of UNCLUSTERED_LABEL, we need an offset
    // that shows where counting real clusters starts from.
    constexpr std::uint32_t LABEL_OFFSET = 1;
}


namespace lidar_course {


// Choice between convex and concave hull
enum HullType
{
    HullTypeConvex,
    HullTypeConcave
};


// A function that multiplexes convex and concave 2D hulls
template<typename T>
auto GenericHull2D(const typename pcl::PointCloud<T>::Ptr& flat_cloud_ptr,
    HullType hull_type, float alpha = 0.9f)
{
    auto flat_hull_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    auto flat_polygons_ptr = std::make_shared<std::vector<pcl::Vertices> >();

    if (hull_type == HullTypeConcave)
    {
        pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
        concave_hull.setInputCloud(flat_cloud_ptr);
        concave_hull.setAlpha(alpha);
        concave_hull.setDimension(2);
        concave_hull.reconstruct(*flat_hull_cloud_ptr, *flat_polygons_ptr);
    }
    else
    {
        pcl::ConvexHull<pcl::PointXYZ> convex_hull;
        convex_hull.setInputCloud(flat_cloud_ptr);
        convex_hull.setDimension(2);
        convex_hull.reconstruct(*flat_hull_cloud_ptr, *flat_polygons_ptr);
    }

    return std::make_tuple(flat_hull_cloud_ptr, flat_polygons_ptr);
}


// This function gets the clustered point cloud with labels and
// selects "top_clusters" biggest clusters from it, and then computes
// a Z-cylindrical convex hull for each cluster.
CloudAndClusterHulls find_primary_clusters(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud,
    size_t top_clusters = 200,
    HullType hull_type = HullTypeConvex)
{
    typedef std::uint32_t label_t;
    typedef std::uint32_t counter_t;

    // Here we obtain a statistics of cluster sizes. Let's store it
    // in a hash map for the sake of speed.
    std::unordered_map<label_t, counter_t> cluster_counts;
    for (const auto& point : *cloud)
    {
        // https://pointclouds.org/documentation/classpcl_1_1_supervoxel_clustering.html
        // Labels for segments start from 1, unlabled points have label 0
        if (point.label != 0)
        {
            cluster_counts[point.label]++;
        }
    }
    std::cout << "num_clusters=" << cluster_counts.size() << std::endl;

    // We need to convert the hash map into a vector for sorting.
    std::vector<std::pair<label_t, counter_t> > counts_vec;
    for (const auto element : cluster_counts)
    {
        counts_vec.push_back(element);
    }

    // Sort the cluster stats descending, preserving the labels. 
    std::sort(counts_vec.begin(), counts_vec.end(), [](auto elem1, auto elem2) {
        return elem1.second > elem2.second;
        });

    // We want to keep only several biggest clusters,
    // all other will be relabled as unlabeled.
    label_t max_clusters = static_cast<label_t>(
        (cluster_counts.size() < top_clusters) ? cluster_counts.size() : top_clusters);

    // Let's create a re-labeling table.
    std::unordered_map<label_t, label_t> remap_from_to;
    for (label_t i = 0; i < max_clusters; i++)
    {
        // std::cout << counts_vec[i].first << " " << counts_vec[i].second << std::endl;
        const auto from_label = counts_vec[i].first;
        remap_from_to[from_label] = i + LABEL_OFFSET; // 0 is reserved for "not clustered"
    }

    auto relabeled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL> >();
    relabeled_cloud->is_dense = false;

    // Create separate point clouds for each cluster.
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds(max_clusters);
    for (auto& cluster_cloud : cluster_clouds)
    {
        cluster_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cluster_cloud->is_dense = false;
    }

    // In one pass over the input cloud, (1) relabel it and split into per-cluster clouds
    for (const auto& point : *cloud)
    {
        const auto old_label = point.label;
        pcl::PointXYZL new_point;
        if (remap_from_to.find(old_label) != remap_from_to.end())
        {
            const auto new_label = remap_from_to[old_label];
            assert(new_label >= LABEL_OFFSET && new_label < max_clusters + LABEL_OFFSET);
            new_point = pcl::PointXYZL(point.x, point.y, point.z, new_label);
            relabeled_cloud->push_back(new_point);

            auto& cluster_cloud_ptr = cluster_clouds[new_label - LABEL_OFFSET];
            cluster_cloud_ptr->push_back(pcl::PointXYZ(point.x, point.y, point.z));
        }
        else
        {
            new_point = pcl::PointXYZL(point.x, point.y, point.z, UNCLUSTERED_LABEL);
            relabeled_cloud->push_back(new_point);
        }
    }

    // Now we can iterate over all clusters and compute convex hulls.
    std::vector<ClusterWithHull> clusters_with_hull;
    for (label_t i_cluster = 0; i_cluster < cluster_clouds.size(); i_cluster++)
    {
        const auto& cluster_cloud_ptr = cluster_clouds[i_cluster];
        if (cluster_cloud_ptr->size() == 0)
        {
            continue;
        }

        // Since our hull will be cylindrical along Z axis, we need to determine
        // min and max Z coordinates that will form bottom and top hull faces.
        float min_z = std::numeric_limits<float>::infinity();
        float max_z = -std::numeric_limits<float>::infinity();
        for (auto& p : *cluster_cloud_ptr)
        {
            min_z = std::min(min_z, p.z);
            max_z = std::max(max_z, p.z);
        }

        // We need to flatten the cloud so that qhull will run
        // a 2D hull extraction in ZY plane.
        auto flat_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        for (const auto& p : *cluster_cloud_ptr)
        {
            flat_cloud_ptr->push_back(pcl::PointXYZ(p.x, p.y, 0.0f));
        }

        // Call flat (2D) hull extraction.
        // std::tie unpacks a tuple.
        auto flat_hull_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        auto flat_polygons_ptr = std::make_shared<std::vector<pcl::Vertices> >();
        std::tie(flat_hull_cloud_ptr, flat_polygons_ptr) =
            GenericHull2D<pcl::PointXYZ>(flat_cloud_ptr, hull_type);

        auto full_hull_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        auto full_polygons_ptr = std::make_shared<std::vector<pcl::Vertices> >();

        // At this stage we need to create a full mesh for visualization.
        // The full mesh consists of a bottom face, a top face and a side.

        // Create a top face, this is straightforward.
        for (const auto& p : *flat_hull_cloud_ptr)
        {
            pcl::PointXYZ p3(p.x, p.y, max_z);
            full_hull_cloud_ptr->push_back(p3);
        }
        if (hull_type == HullTypeConvex)
        {
            for (const auto& poly : *flat_polygons_ptr)
            {
                full_polygons_ptr->push_back(poly);
            }
        }

        // For the bottom face we need to re-index the vertices of polygons.
        const auto offset = static_cast<std::uint32_t>(flat_hull_cloud_ptr->size());
        for (const auto& p : *flat_hull_cloud_ptr)
        {
            pcl::PointXYZ p3(p.x, p.y, min_z);
            full_hull_cloud_ptr->push_back(p3);
        }
        if (hull_type == HullTypeConvex)
        {
            for (const auto& poly : *flat_polygons_ptr)
            {
                pcl::Vertices poly_offset;
                for (const auto& vertex_index : poly.vertices)
                {
                    poly_offset.vertices.push_back(vertex_index + offset);
                }
                full_polygons_ptr->push_back(poly_offset);
            }
        }

        // For the sides we are going to reuse the existing points,
        // and add extra vertices to the vertices array.
        for (std::uint32_t i_point = 0; i_point < flat_hull_cloud_ptr->size(); i_point++)
        {
            const auto next_point = (i_point + 1) % offset;
            pcl::Vertices simplex_1;
            simplex_1.vertices = std::vector<std::uint32_t> {
                i_point,
                next_point,
                i_point + offset};
            full_polygons_ptr->push_back(simplex_1);

            pcl::Vertices simplex_2;
            simplex_2.vertices = std::vector<std::uint32_t> {
                next_point,
                next_point + offset,
                i_point + offset};
            full_polygons_ptr->push_back(simplex_2);
        }

        // Finally fill the resulting struct for the cluster and append it
        // to the total list.
        ClusterWithHull cluster_with_hull {
            i_cluster + LABEL_OFFSET,
            cluster_cloud_ptr,
            full_hull_cloud_ptr,
            full_polygons_ptr
        };
        clusters_with_hull.push_back(cluster_with_hull);
    }

    return {relabeled_cloud, clusters_with_hull};
}


} // namespace lidar_course
