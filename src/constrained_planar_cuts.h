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

#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/cpc_segmentation.h>


namespace lidar_course {


// https://openaccess.thecvf.com/content_cvpr_2015/papers/Schoeler_Constrained_Planar_Cuts_2015_CVPR_paper.pdf


template<typename PointT>
auto constrained_planar_cuts_segmentation(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr)
{
    ///  Default values of parameters before parsing
    
    // Supervoxel Stuff
    float voxel_resolution = 0.20f; //0.15f
    float seed_resolution = 0.03f;
    float color_importance = 0.0f;
    float spatial_importance = 1.0f;
    float normal_importance = 4.0f;
    bool use_single_cam_transform = false;
    bool use_supervoxel_refinement = false;

    // LCCPSegmentation Stuff
    float concavity_tolerance_threshold = 10;
    float smoothness_threshold = 0.1;
    uint32_t min_segment_size = 0;
    bool use_extended_convexity = false;
    bool use_sanity_criterion = false;

    // CPCSegmentation Stuff
    float min_cut_score = 0.16;
    unsigned int max_cuts = 25;
    unsigned int cutting_min_segments = 400;
    bool use_local_constrain = false;
    bool use_directed_cutting = false;
    bool use_clean_cutting = false;
    unsigned int ransac_iterations = 10000;

    // Segmentation Stuff
    unsigned int k_factor = 0;
    if (use_extended_convexity)
        k_factor = 1;

    max_cuts = 0;

    float textcolor = 1;

    // Preparation of Input: Supervoxel Oversegmentation
    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform(use_single_cam_transform);
    super.setInputCloud(input_cloud_ptr);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);
    std::map<uint32_t, typename pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

    super.extract(supervoxel_clusters);

    if (use_supervoxel_refinement)
    {
        super.refineSupervoxels(2, supervoxel_clusters);
    }

    std::multimap<uint32_t, uint32_t>supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    // Get the cloud of supervoxel centroid with normals and the colored cloud
    // with supervoxel coloring(this is used for visulization)
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud =
        pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(supervoxel_clusters);

    typename pcl::CPCSegmentation<PointT> cpc;
    cpc.setConcavityToleranceThreshold(concavity_tolerance_threshold);
    cpc.setSanityCheck(use_sanity_criterion);
    cpc.setCutting(max_cuts, cutting_min_segments, min_cut_score,
        use_local_constrain, use_directed_cutting, use_clean_cutting);
    cpc.setRANSACIterations(ransac_iterations);
    cpc.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
    cpc.setKFactor(k_factor);
    cpc.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    cpc.setMinSegmentSize(min_segment_size);
    cpc.segment();

    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr cpc_labeled_cloud = sv_labeled_cloud->makeShared();
    cpc.relabelCloud(*cpc_labeled_cloud);

    return cpc_labeled_cloud;
}


} // namespace lidar_course
