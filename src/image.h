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

#include <string>
#include <memory>

#define cimg_use_png 1
#include "CImg/CImg.h"


namespace lidar_course {


class Image
{
    cil::CImg<unsigned char> m_image;

public:
    Image(std::string path) :
        m_image(path.c_str())
    {
    }

    const unsigned char* ptr()
    {
        return m_image.data(0, 0);
    }
    
    int width()
    {
        return m_image.width();
    }

    int height()
    {
        return m_image.height();
    }
};


std::unique_ptr<Image> load_image(std::string path)
{
    std::unique_ptr<Image> result;
    try
    {
        result = std::make_unique<Image>(path);
    }
    catch(const std::exception& e)
    {
        // std::cerr << e.what() << '\n';
    }

    return result;
}


} // namespace lidar_course
