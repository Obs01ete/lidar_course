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


#include <stdlib.h>
#include <thread>
#include <chrono>

#include <pcl/io/pcd_io.h>

#include "processor.h"
#include "visualizer.h"


// This application performs lidar_course::Processor
// computations over a single .pcd point cloud.
int main(int argc, char** argv)
{
    using namespace lidar_course;

    if (argc < 2)
    {
        pcl::console::print_info(
\
        "\n\
-- Process one point cloud in PCD format -- :\n\
\n\
Syntax: %s input.pcd\n\
\n\
",
        argv[0]);
        return EXIT_FAILURE;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    std::string pcd_filename = argv[1];

    pcl::PCLPointCloud2 input_pointcloud2;
    if (pcl::io::loadPCDFile(pcd_filename, input_pointcloud2))
    {
        std::cout << "Cannot read input point cloud " << pcd_filename << std::endl;
        return EXIT_FAILURE;
    }

    pcl::fromPCLPointCloud2(input_pointcloud2, *input_cloud_ptr);

    auto processor_params = std::make_shared<ProcessorParams>(1, false, 1, true);
    lidar_course::Processor processor(processor_params);
    auto cloud_and_clusters = processor.process(input_cloud_ptr, std::unique_ptr<kitti_parser::gpsimu_t>());

    lidar_course::Visualizer viewer(processor_params);

    while (!viewer.was_stopped())
    {
        viewer.show(cloud_and_clusters);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return EXIT_SUCCESS;
}
