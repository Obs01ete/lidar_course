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


namespace lidar_course {


namespace {
    const size_t MAX_CLOUDS = 10;
    const size_t MAX_DECIMATION = 10;
}


// This structure keeps all the parameters of processing that are
// supposed to be controllable from the GUI.
struct ProcessorParams
{
    std::atomic<size_t> m_decimation_coef;
    std::atomic<bool> m_remove_ground;
    std::atomic<size_t> m_num_clouds;
    std::atomic<bool> m_do_clusterize;

    ProcessorParams(size_t decimation_coef,
                    bool remove_ground,
                    size_t num_clouds,
                    bool do_clusterize) :

        m_remove_ground(remove_ground),
        m_do_clusterize(do_clusterize)
    {
        setDecimationCoef(decimation_coef);
        setNumClouds(num_clouds);
    }

    void setDecimationCoef(size_t decimation_coef)
    {
        m_decimation_coef = ensureRange(decimation_coef, 1, MAX_DECIMATION);
    }

    void setNumClouds(size_t num_clouds)
    {
        m_num_clouds = ensureRange(num_clouds, 1, MAX_CLOUDS);
    }

private:
    size_t ensureRange(size_t val, size_t min, size_t max)
    {
        if (val > max)
        {
            val = max;
        }

        if (val < min)
        {
            val = min;
        }

        return val;
    }
};


} // namespace lidar_course