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


#include "visualizer.h"


namespace lidar_course {


Visualizer::Visualizer(std::shared_ptr<ProcessorParams>& param_ptr) :
    m_params(param_ptr),
    m_viewer(),
    m_show_help(true),
    m_is_running(true)
{
    m_viewer = std::make_shared<pcl::visualization::PCLVisualizer>("LiDAR view");

    m_viewer->setSize(1600, 900);
    m_viewer->setBackgroundColor(0, 0, 0);
    m_viewer->addCoordinateSystem();
    m_viewer->registerKeyboardCallback(keyboardEventOccurred, this);
    m_viewer->setCameraPosition(
        -20, -2, 4,
        0, 0, 1.5,
        0, 0, 1);
    m_viewer->setCameraClipDistances(0.1, 1000);
    m_viewer->updateCamera();
}


void Visualizer::show(const CloudAndClusterHulls& cloud_and_clusters)
{
    m_viewer->removeAllPointClouds();
    m_viewer->removeAllShapes();

    m_viewer->addPointCloud(cloud_and_clusters.cloud, "labeled_cloud");

    for (const auto& cluster_with_hull : cloud_and_clusters.clusters_with_hull)
    {
        const auto cluster_name = "cluster_" + std::to_string(cluster_with_hull.cluster_id);

        if (cluster_with_hull.hull_vertices->size() > 0)
        {
            m_viewer->addPolygonMesh<pcl::PointXYZ>(
                cluster_with_hull.hull_cloud, *cluster_with_hull.hull_vertices, cluster_name);

            m_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY, 0.25, cluster_name);
        }
    }

    if (m_show_help)
    {
        printText(m_viewer);
    }
    else
    {
        m_viewer->addText("Press d to show help", 5, 15, 12, 1, 1, 1, "help_text");
    }

    m_viewer->spinOnce();
}


bool Visualizer::was_stopped()
{
    return m_viewer->wasStopped();
}

bool Visualizer::is_running()
{
    return m_is_running;
}


void Visualizer::keyboardEventOccurred(
    const pcl::visualization::KeyboardEvent& event_arg,
    void* pv_this)
{
    Visualizer* pthis = (Visualizer*)pv_this;

    int key = event_arg.getKeyCode();

    if (event_arg.keyUp())
    {
        switch (key)
        {
        case (int) '1':
            pthis->m_params->setNumClouds(pthis->m_params->m_num_clouds - 1);
            break;
        case (int) '2':
            pthis->m_params->setNumClouds(pthis->m_params->m_num_clouds + 1);
            break;
        case (int) '3':
            pthis->m_params->m_remove_ground = !pthis->m_params->m_remove_ground;
            break;
        case (int) '4':
            pthis->m_params->m_do_clusterize = !pthis->m_params->m_do_clusterize;
            break;
        case (int) '6':
            pthis->m_params->setDecimationCoef(pthis->m_params->m_decimation_coef - 1);
            break;
        case (int) '7':
            pthis->m_params->setDecimationCoef(pthis->m_params->m_decimation_coef + 1);
            break;
        case (int) 'd':
        case (int) 'D':
            pthis->m_show_help = !pthis->m_show_help;
            break;
        case (int) 's':
        case (int) 'S':
            pthis->m_is_running = !pthis->m_is_running;
            break;
        default:
            break;
        }
    }
}


void Visualizer::printText(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_arg)
{
    std::string on_str = "ON";
    std::string off_str = "OFF";
    int font_size = 13;
    int space = 15;
    int top = 120;

    viewer_arg->addText("Press (1-n) to show different elements",
        5, top, font_size+2, 1, 1, 1, "hud_text");

    std::string temp;

    top -= space;
    temp = "(1-2) Decrease-increase # of aggregated clouds";
    viewer_arg->addText(temp, 5, top, font_size, 1, 1, 1, "agg_text");

    top -= space;
    temp = "(3) Remove ground";
    viewer_arg->addText(temp, 5, top, font_size, 1, 1, 1, "ground_text");

    top -= space;
    temp = "(4) Extract clusters";
    viewer_arg->addText(temp, 5, top, font_size, 1, 1, 1, "seg_text");

    top -= space;
    temp = "(6-7) Decrease-increase decimation coefficient";
    viewer_arg->addText(temp, 5, top, font_size, 1, 1, 1, "decim_text");

    top -= space;
    temp = "(S) Run / stop";
    viewer_arg->addText(temp, 5, top, font_size, 1, 1, 1, "run_text");

    top -= space;
    temp = "(Q) Quit";
    viewer_arg->addText(temp, 5, top, font_size, 1, 1, 1, "quit_text");
}


} // namespace lidar_course
