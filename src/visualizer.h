
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

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>

#include "interface_types.h"
#include "processor_params.h"


namespace lidar_course {


class Visualizer
{
public:
    std::shared_ptr<ProcessorParams> m_params;

private:
    pcl::visualization::PCLVisualizer::Ptr m_viewer;

    bool m_show_help;
    bool m_is_running;

public:
    Visualizer(std::shared_ptr<ProcessorParams>& param_ptr);
    void show(const CloudAndClusterHulls& cloud_and_clusters);
    bool was_stopped();
    bool is_running();

private:
    static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event_arg, void* pv_this);
    static void printText(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_arg);
    static void removeText(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_arg);
};


} // namespace lidar_course
