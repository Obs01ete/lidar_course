/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <cassert>
#include <queue>
#include <vector>
#include <chrono>

// #include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>


namespace lidar_course {


template<typename PointT>
auto dbscan_segmentation(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr)
{
    std::cout << "inp PC size " << input_cloud_ptr->size() << std::endl;

    // float resolution = 128.0f;
    // pcl::octree::OctreePointCloudSearch<PointT> tree(resolution);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;

    {
        auto t1 = std::chrono::steady_clock::now();
        tree.setInputCloud(input_cloud_ptr);
        // tree.addPointsFromInputCloud();
        auto t2 = std::chrono::steady_clock::now();
        auto d_milli = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
        std::cout << "tree " << d_milli << std::endl;
    }

    float eps = 0.5f;
    size_t minPts = 5;

    const size_t UNDEFINED = 0;
    const size_t NOISE = 1;
    std::vector<size_t> labels(input_cloud_ptr->size(), UNDEFINED);
    size_t currClusterId = NOISE + 1;
    std::vector<size_t> quantsList;

    {
        auto t1 = std::chrono::steady_clock::now();

        for (size_t queryIdx = 0; queryIdx < input_cloud_ptr->size(); queryIdx++)
        {
            if (labels[queryIdx] != UNDEFINED)
            {
                continue;
            }
            
            const auto pointP = (*input_cloud_ptr)[queryIdx];

            std::vector<int> foundIndicesP;
            std::vector<float> foundSqDistP;
            tree.radiusSearch(pointP, eps, foundIndicesP, foundSqDistP);
            quantsList.push_back(foundIndicesP.size());
            if (foundIndicesP.size() < minPts)
            {
                labels[queryIdx] = NOISE;
                continue;
            }

            labels[queryIdx] = currClusterId;
            std::queue<size_t> bfsQueue;
            for (const auto foundIdxP : foundIndicesP)
            {
                assert(foundIdxP >= 0);
                if (foundIdxP == queryIdx)
                {
                    continue;
                }
                bfsQueue.push(static_cast<size_t>(foundIdxP));
            }

            while (bfsQueue.size() > 0)
            {
                size_t indexQ = bfsQueue.front();
                bfsQueue.pop();
                const auto pointQ = (*input_cloud_ptr)[indexQ];
                if (labels[indexQ] == NOISE)
                {
                    labels[indexQ] = currClusterId;
                }
                if (labels[indexQ] != UNDEFINED)
                {
                    continue;
                }
                labels[indexQ] = currClusterId;

                std::vector<int> foundIndicesQ;
                std::vector<float> foundSqDistQ;
                tree.radiusSearch(pointQ, eps, foundIndicesQ, foundSqDistQ);
                quantsList.push_back(foundIndicesQ.size());
                if (foundIndicesQ.size() >= minPts)
                {
                    for (const auto foundIdxQ : foundIndicesQ)
                    {
                        assert(foundIdxQ >= 0);
                        bfsQueue.push(static_cast<size_t>(foundIdxQ));
                    }
                }
            }

            currClusterId++;
        }

        auto t2 = std::chrono::steady_clock::now();
        auto d_milli = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
        std::cout << "dbscan " << d_milli << std::endl;
    }

    std::cout << "num searches " << quantsList.size() << " avg num neigh "
        << std::accumulate(quantsList.begin(), quantsList.end(), 0) / quantsList.size()
        << std::endl;

    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud(new pcl::PointCloud<pcl::PointXYZL>());
    for (size_t i = 0; i < input_cloud_ptr->size(); i++)
    {
        const auto& p = (*input_cloud_ptr)[i];
        const auto origLabel = labels[i];
        assert(origLabel != UNDEFINED);
        static_assert(NOISE == 1);
        const size_t label = origLabel - NOISE;
        pcl::PointXYZL labeledPoint(p.x, p.y, p.z, label);
        labeled_cloud->push_back(labeledPoint);
    }

    return labeled_cloud;
}


} // namespace lidar_course
