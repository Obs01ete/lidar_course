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
#include <iostream>
#include <vector>
#include <map>
#include <string>

#include "gpsimu_t.h"


namespace lidar_course {


typedef std::vector<kitti_parser::gpsimu_t> GpsImu;


// This fuction loads Kitti's .txt files with OXTS localization data.
// We need localization to perform point cloud temporal aggreagation.
GpsImu parse_oxts(std::string path)
{
    GpsImu list;

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
            // std::cout << result.size() << std::endl;
            if (result.size() == 30)
            {
                std::vector<double> elems;
                for (int i = 0; i < 30; i++)
                {
                    elems.push_back(std::stof(result[i]));
                }
                kitti_parser::gpsimu_t gpsimu = {
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
                    elems[12],
                    elems[13],
                    elems[14],
                    elems[15],
                    elems[16],
                    elems[17],
                    elems[18],
                    elems[19],
                    elems[20],
                    elems[21],
                    elems[22],
                    elems[23],
                    elems[24],
                    (int)elems[25],
                    (int)elems[26],
                    (int)elems[27],
                    (int)elems[28],
                    (int)elems[29]
                };
                list.push_back(gpsimu);
            }
        }
        infile.close();
    }
    return list;
}


} // namespace lidar_course
