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
#include <future>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/image_viewer.h>

#include "visualizer.h"
#include "processor.h"
#include "getdir.h"
#include "load_bin.h"
#include "parse_calib.h"
#include "parse_oxts.h"
#include "image.h"
#include "interface_types.h"


// This is the main application to process a Kitti tracking sequence
// as a whole and render processing results in a GUI window.
int main(int argc, char** argv)
{
    using namespace lidar_course;

    // The first argument is a path to training/testing subfolder of
    // the Kitti tracking dataset.
    // The second argument is a name of the sequence which is 4 digits.

    if (argc < 3)
    {
        pcl::console::print_info(
            "\nSyntax: %s /data/kitti/tracking/training/ 0000\n\n",
            argv[0]);
        return EXIT_FAILURE;
    }

    std::string data_root = argv[1];
    std::string sequence_name = argv[2];

    // Here we enumerate all the .bin files in velodyne folder
    std::string velodyne_root_dir = data_root + "/velodyne";
    std::string velodyne_dir = velodyne_root_dir + "/" + sequence_name;
    std::vector<std::string> filenames;
    getdir(velodyne_dir, filenames);

    // We need to create an index of all frames (scans) so that we
    // can process them as a sequence and provide an animated visualization.
    std::vector<std::string> frame_ids;
    for (const auto& filename : filenames)
    {
        if (boost::algorithm::ends_with(filename, ".bin"))
        {
            size_t last_index = filename.find_last_of("."); 
            std::string name = filename.substr(0, last_index);
            frame_ids.push_back(name);
        }
    }
    // The files may not be grabbed in a sorted manner from the file system.
    // We need to sort them to maintain the right sequence of scans.
    std::sort(frame_ids.begin(), frame_ids.end());
    std::cout << frame_ids.size() << std::endl;

    // We will visualize images apart from LiDAR scans to help understand
    // how the scene looks like from the camera.
    std::string image_dir = data_root + "/image_02/" + sequence_name;

    // Calibration matrices are already there in the dataset, just load them
    std::string calib_path = data_root + "/calib/" + sequence_name + ".txt";
    auto calib = parse_calib(calib_path);
    std::cout << calib.size() << std::endl;
    std::cout << calib["Tr_imu_velo"] << std::endl;

    // GPS/IMU localization is what is also prepared in the dataset.
    // We will need it for point cloud aggregation over multiple scans.
    std::string oxts_path = data_root + "/oxts/" + sequence_name + ".txt";
    auto oxts_list = parse_oxts(oxts_path);
    std::cout << oxts_list.size() << std::endl;

    // Here we instantiate processor parameters separately, since we are going
    // to pass them to both processor and visualizer. In this way we can
    // change processing parameters with the hotkeys.
    const size_t decimation_coef = 5;
    const bool remove_ground = true;
    const size_t num_clouds_to_aggregate = 2;
    const bool do_clusterize = true;
    auto processor_params = std::make_shared<ProcessorParams>(
        decimation_coef,
        remove_ground,
        num_clouds_to_aggregate,
        do_clusterize);

    // Here we instantiate our processing class.
    Processor processor(processor_params);

    // This is instantiation of the point cloud visualizer class. 
    Visualizer cloud_viewer(processor_params);

    // We are going to render images in a separate window.
    auto image_viewer =
        std::make_shared<pcl::visualization::ImageViewer>("Front camera view");

    // This is the loop for repeatitive processing of the same sequence.
    // Once the sequence is done being processed, the cycle starts over again.
    while (true)
    {
        bool do_break = false;

        // On every pass through the sequence we need to reset the processor state
        processor.reset();

        int frame_idx = 0;
        CloudAndClusterHulls cloud_and_clusters{};
        std::unique_ptr<Image> p_image;
        
        // In this loop the frames are processed and visualized one by one
        while (true)
        {
            auto start_time = std::chrono::system_clock::now();
            
            // This condition allows us to stop and resume playback of
            // the sequence with S key
            if (cloud_viewer.is_running() || frame_idx == 0)
            {
                const auto frame_id = frame_ids[frame_idx];

                // We are going to load the image in a separate thread
                // in order to not slowdown the main thread
                std::string png_path = image_dir + "/" + frame_ids[frame_idx] + ".png";
                std::future<std::unique_ptr<Image> > fut_p_image =
                    std::async(std::launch::async, load_image, png_path);

                // Here we load the current .bin file with LiDAR data
                std::string bin_path = velodyne_dir + "/" + frame_id + ".bin";
                const auto input_cloud_ptr = load_bin(bin_path);

                // And select the corresponding localization record
                const auto oxts = std::unique_ptr<kitti_parser::gpsimu_t>(
                    new kitti_parser::gpsimu_t(oxts_list[frame_idx]));

                // Here we go, run our processing function
                cloud_and_clusters = processor.process(input_cloud_ptr, oxts);

                // We need to acquire the loaded image from another thread
                p_image = fut_p_image.get();

                frame_idx++;
                if (frame_idx >= oxts_list.size())
                {
                    break;
                }
            }

            // We want to keep track of the processing time
            auto end_time = std::chrono::system_clock::now();
            std::chrono::duration<double> processing_time = end_time - start_time;
            int processing_time_ms = int(processing_time.count()*1000);
            std::cout << "Processing time " << processing_time_ms << " ms" << std::endl;

            // Call rendering of the LiDAR visualization window
            cloud_viewer.show(cloud_and_clusters);

            // Call rendering of the image visualization window
            // (if the imame is successfully loaded)
            if (p_image)
            {
                image_viewer->addRGBImage(p_image->ptr(),
                    p_image->width(), p_image->height());
                image_viewer->spinOnce();
            }

            // If the processing is happening faster than real time (10 fps),
            // sleep until 100 ms have passed.
            int reference_time_ms = 100;
            int pad_time_ms = std::max(0, reference_time_ms-processing_time_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(pad_time_ms));

            if (cloud_viewer.was_stopped())
            {
                do_break = true;
                break;
            }

        }

        if (do_break)
        {
            break;
        }
    }

    return EXIT_SUCCESS;
}
