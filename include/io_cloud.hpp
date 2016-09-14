/**
*** header for freenect2 grabber for saving or loading cloud point *.ply files ***
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


#pragma once

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>


namespace f2g {

    class io_cloud {

    public:

        template <typename Tcloud>
        int savePLYCloud(const std::string &filename, const Tcloud &cloud, bool binaryformat, bool useCam);

        template <typename Tcloud>
        int savePCDCloud(const std::string &filename, const Tcloud &cloud, bool binaryformat, bool useCam);

        template <typename Tcloud>
        int loadPLYCloud(const std::string &filename, const Tcloud &loadCloud);

        template <typename Tcloud>
        int loadPCDCloud(const std::string &filename, const Tcloud &loadCloud);

        template <typename Tcloud>
        void convertCloud(const std::string &filename, const std::string &convertedFile, const Tcloud &cloud, bool binformat);

        template <typename Tcloud>
        void savePCDCloud(bool safe, const std::string &filename, const Tcloud &cloud, bool binaryformat);

        template <typename Tcloud>
        void savePLYCloud(bool safe, const std::string &filename, const Tcloud &cloud, bool binaryformat);

    private:

        int errNo_;

        pcl::PLYWriter plywriter_;
        pcl::PCDWriter pcdwriter_;

    };

}
