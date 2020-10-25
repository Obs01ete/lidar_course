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

#include <fstream>
#include <vector>
#include <map>
#include <string>

#include <Eigen/Dense>


namespace lidar_course {


// Every container that stores Eigen objects must have an aligned_allocator specified.
typedef std::map<std::string, Eigen::Matrix4f, std::less<std::string>,
    Eigen::aligned_allocator<std::pair<std::string, Eigen::Vector4f> > > CalibMap;


// Parses Kitti calibration file format into Eigen::Matrix4f
CalibMap parse_calib(std::string path)
{
    CalibMap map;

    std::ifstream infile(path);
    if (infile.is_open())
    {
        std::string line;
        while (std::getline(infile, line))
        {
            std::istringstream iss(line);
            std::vector<std::string> result;
            for (std::string s; iss >> s; )
            {
                result.push_back(s);
            }
            if (result.size() == 13)
            {
                auto name = result[0];
                std::vector<float> elems;
                for (int i = 1; i <= 12; i++)
                {
                    elems.push_back(std::stof(result[i]));
                }
                Eigen::Matrix4f mat;
                mat <<
                    elems[0],
                    elems[1],
                    elems[2],
                    elems[3],
                    elems[4],
                    elems[5],
                    elems[6],
                    elems[7],
                    elems[8],
                    elems[9],
                    elems[10],
                    elems[11],
                    0, 0, 0, 1;
                map[name] = mat;
            }
        }
        infile.close();
    }
    return map;
}


} // namespace lidar_course
