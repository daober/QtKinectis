/**
*** header for freenect2 grabber for additional helper functions ***

Copyright 2016, Daniel Obermaier

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author Daniel Obermaier
*/

//include own headers first
#pragma once

#include "grabber.hpp"
#include "logger.hpp"
#include "eventlistener.hpp"

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>

#include <chrono>



namespace f2g {

    class grabber_impl{

    public:

        int processColorizedPointCloud(f2g::proc pl, bool setSize = true, int xw = 1280, int yw = 768);
        int processUncolorizedPointCloud(f2g::proc pl, bool setSize = true, int xw = 1280, int yw = 768);

        cv::Mat getColorMat();
        cv::Mat getDepthMat();

    private:

        bool isErr_;

        pcl::console::TicToc tictoc_;

        cv::Mat color_;
        cv::Mat depth_;
        cv::Mat ir_;
    };

}
