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

#include <vector>
#include <memory>

#include <pcl/common/common.h>


namespace lidar_course {


// This struct keeps the description of everything that relates to a cluster.
struct ClusterWithHull
{
    // Unique identifier, starting from 1 as 0 is reserved for "unclustered"
    std::uint32_t cluster_id;
    // The entire point cloud of the cluster
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
    // Exclusively points that belong to the hull.
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr hull_cloud;
    // The list of triangles to render the hull,
    // as indices of points from hull_cloud
    std::shared_ptr<std::vector<pcl::Vertices> > hull_vertices;
};


// This is a convenience struct that holds the labeled cloud and
// a list of clusters and their convex hulls.
struct CloudAndClusterHulls
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud;
    std::vector<ClusterWithHull> clusters_with_hull;
};


} // namespace lidar_course
