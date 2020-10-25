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


#include <iostream>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


int main()
{
    const std::string infile = "sample_clouds/000008.bin";
    const std::string outfile = "sample_clouds/000008.pcd";

	std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
	if (!input.good())
	{
		std::cerr << "Could not read file: " << infile << std::endl;
		return EXIT_FAILURE;
	}
	input.seekg(0, std::ios::beg);

	auto points = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();

	for (int i = 0; input.good() && !input.eof(); i++)
	{
		pcl::PointXYZI point;
		input.read((char*)&point.x, 3*sizeof(float));
		input.read((char*)&point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();

	std::cout << "points->size() = " << points->size() << std::endl;

	std::cout << "Read KTTI point cloud with " << points->size() <<
	    " points, writing to " << outfile << std::endl;
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI>(outfile, *points, false);

    return EXIT_SUCCESS;
}
