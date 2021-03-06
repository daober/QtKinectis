/**
*** source for freenect2 grabber for saving or loading cloud point *.ply files ***
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


#include "iocloud.hpp"

#include <chrono>
#include <iostream>



f2g::saveHelper::saveHelper(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, bool binformat, bool useCam, f2g::grabber &grab) :
                                                            cloud_(cloud), binformat_(binformat), useCam_(useCam), grabber_(grab) {



}

f2g::saveHelper::saveHelper(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud, bool binformat, bool useCam, f2g::grabber &grab) :
                                                            cloudXYZ_(cloud), binformat_(binformat), useCam_(useCam), grabber_(grab) {



}


template <typename Tcloud>
int f2g::iocloud::savePLYCloud(const std::string &filename, const Tcloud &cloud, bool binaryformat, bool useCam){
    std::cout<<"saving cloud as *.ply file"<<std::endl;

    //TODO: should have a own button (implementing qt + custom pcl viewer)
    std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();

    std::string tnow = std::to_string((double) std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count());

    plywriter_.write(filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binaryformat);

    std::cout << "saved: " << "cloud " + tnow + ".ply" << std::endl;

    return (0);
}



template <typename Tcloud>
int f2g::iocloud::savePCDCloud(const std::string &filename, const Tcloud &cloud, bool binaryformat, bool useCam){
    std::cout<<"saving cloud as *.pcd file"<<std::endl;

    //TODO: should have own button as well
    std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();

    std::string tnow = std::to_string((double) std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count());

    pcdwriter_.write(filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binaryformat);

    std::cout << "saved: " << "cloud " + tnow + ".pcd" << std::endl;

    return (0);
}



template <typename Tcloud>
int f2g::iocloud::loadPLYCloud(const std::string &filename, const Tcloud &loadCloud){
    std::cout<<"loading cloud as *.ply file"<<std::endl;

    return (0);
}



template <typename Tcloud>
int f2g::iocloud::loadPCDCloud(const std::string &filename, const Tcloud &loadCloud){
    std::cout<<"loading cloud as *.ply file"<<std::endl;

    return (0);
}



template <typename Tcloud>
void f2g::iocloud::convertCloud(const std::string &filename, const std::string &convertedFile, const Tcloud &cloud, bool binformat){

    //TODO: empty body!
}


template <typename Tcloud>
void f2g::iocloud::savePLYCloud(bool safe, const std::string &filename, const Tcloud &cloud, bool binaryformat){

    if(safe){
        std::cout<<"saving cloud as *.ply file"<<std::endl;

        //TODO: should have a own button (implementing qt + custom pcl viewer)
        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();

        std::string tnow = std::to_string((double) std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count());

        plywriter_.write(filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binaryformat);

        std::cout << "saved: " << "cloud " + tnow + ".ply" << std::endl;
    }
}


template <typename Tcloud>
void f2g::iocloud::savePCDCloud(bool safe, const std::string &filename, const Tcloud &cloud, bool binaryformat){

    if(safe){
        std::cout<<"saving cloud as *.pcd file"<<std::endl;

        //TODO: should have own button as well
        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();

        std::string tnow = std::to_string((double) std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count());

        pcdwriter_.write(filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binaryformat);

        std::cout << "saved: " << "cloud " + tnow + ".pcd" << std::endl;
    }
}
