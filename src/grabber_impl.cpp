/**
***class definition for freenect2 grabber for additional helper functions ***

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

#include "grabber_impl.hpp"
#include "eventlistener.hpp"
#include "io_cloud.hpp"



int f2g::grabber_impl::processPointCloud(f2g::proc pl, bool colorVwr, bool pclVwr){

    int errNo = 0;

    std::cout<< "Processing Point Cloud..." <<std::endl;

    std::vector<int> iter_ply;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> mCloud;

    boost::shared_ptr<f2g::grabber> f2grab(new f2g::grabber(pl));

    mCloud = f2grab->getPointCloud();

    mCloud->sensor_orientation_.w() = 0.0f;
    mCloud->sensor_orientation_.x() = 1.0f;
    mCloud->sensor_orientation_.y() = 0.0f;
    mCloud->sensor_orientation_.z() = 0.0f;

    f2grab->setRGBViewer(colorVwr);
    f2grab->setPCLViewer(pclVwr);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Kinectv2 3D Viewer"));

    viewer->registerKeyboardCallback(f2g::eventlistener::pclKeyboardEvent, (void*)viewer.get());


    if(f2grab->getRGBViewer() || f2grab->getPCLViewer()){
        initializeViewers(mCloud, f2grab, viewer, false);
    }

    f2grab->shutdown();

    return(errNo);
}



//template method creation will follow...
template<typename Tcloud, typename Tgrabber>
int f2g::grabber_impl::initializeViewers(Tcloud cloud, Tgrabber f2grab, boost::shared_ptr<pcl::visualization::PCLVisualizer> vwr, bool setSize, int xw, int yw, bool showFPS){

    int errNo = 0;

    vwr->setBackgroundColor(0.0f, 0.0f, 0.0f);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

    //eigen error happens on declaration
    //boost::shared_ptr<f2g::io_cloud> io(new f2g::io_cloud);

    //TODO: aligned color images need to be aligned
    if(setSize){
        double posx = 0.0;
        double posy = 0.0;
        double posz = -12.0;

        double up_x = 0.0;
        double up_y = 0.0;
        double up_z = 0.0;

        vwr->spinOnce();
        vwr->setSize(xw, yw);
        vwr->setCameraPosition(posx, posy, posz, up_x, up_y, up_z);
    }

    vwr->setShowFPS(showFPS);

    vwr->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    vwr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    while(!vwr->wasStopped()){
        vwr->spinOnce();

        std::chrono::high_resolution_clock::time_point timeNow = std::chrono::high_resolution_clock::now();
        f2grab->getColorDepthAligned(color_, depth_, cloud);

        if(f2grab->getRGBViewer()){
            cv::imshow("color", color_);
            char cv_event =(char) cv::waitKey(10);     //react after 10 msec and cast to char
            f2g::eventlistener::KeyboardInputEvent(cv_event);
        }

        std::chrono::high_resolution_clock::time_point timePost = std::chrono::high_resolution_clock::now();
        std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(timePost-timeNow).count() * 1000 << std::endl;

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

        vwr->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    }

    return (errNo);
}


void f2g::grabber_impl::showUsage(){

        std::cout << "Syntax is: progname [0|1|2|3] correspond to CPU, OPENCL, OPENGL, (CUDA will follow)" << std::endl;
        std::cout << "Press 'ESC' to close OpenCV window" << std::endl;
        exit(-1);
//    std::cout << "Press \'s\' to store a cloud" << std::endl;
//    std::cout << "Press \'x\' to store the calibrations." << std::endl;
}


cv::Mat f2g::grabber_impl::getColorMat(){
    return color_;
}

cv::Mat f2g::grabber_impl::getDepthMat(){
    return depth_;
}
