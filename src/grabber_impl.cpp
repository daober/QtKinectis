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
#include "iocloud.hpp"

#include <Eigen/Core>



int f2g::grabber_impl::processColorizedPointCloud(f2g::proc pl, bool setSize, int xw, int yw){

    int errNo = 0;

    bool showFPS = true;

    std::vector<int> iter_ply;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> mCloud;

    f2g::grabber grab(pl, false);       //args:(pipeline, mirror)

    mCloud = grab.createColorizedPointCloud();

    mCloud->sensor_orientation_.w() = 0.0f;
    mCloud->sensor_orientation_.x() = 1.0f;
    mCloud->sensor_orientation_.y() = 0.0f;
    mCloud->sensor_orientation_.z() = 0.0f;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setBackgroundColor(0.0f, 0.0f, 0.0f);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(mCloud);

    if(setSize){
        double posx = 0.0;
        double posy = 0.0;
        double posz = -12.0;

        double up_x = 0.0;
        double up_y = 0.0;
        double up_z = 0.0;

        viewer->spinOnce();
        viewer->setSize(xw, yw);
        viewer->setCameraPosition(posx, posy, posz, up_x, up_y, up_z);
    }

    viewer->setShowFPS(showFPS);
    viewer->addPointCloud<pcl::PointXYZRGB>(mCloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    f2g::saveHelper save(mCloud, false, false, grab);

    viewer->registerKeyboardCallback(f2g::eventlistener::pclSaveEvent, (void*) &save);
    viewer->registerKeyboardCallback(f2g::eventlistener::pclMiscEvent, (void*) &viewer);

    while(!viewer->wasStopped()){
        viewer->spinOnce();

        std::chrono::high_resolution_clock::time_point timeNow = std::chrono::high_resolution_clock::now();
        grab.getColorDepthAligned(color_, depth_, mCloud);

        /*opencv window block start*/
        //cv::imshow("color", color_);
        //char cv_event =(char) cv::waitKey(10);     //react after 10 msec and cast to char
        //f2g::eventlistener::KeyboardInputEvent(cv_event);
        /*opencv window block end*/

        std::chrono::high_resolution_clock::time_point timePost = std::chrono::high_resolution_clock::now();
        std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(timePost-timeNow).count() * 1000 << std::endl;

        viewer->updatePointCloud<pcl::PointXYZRGB> (mCloud, rgb, "sample cloud");
    }

    grab.shutdown();

    return(errNo);
}


int f2g::grabber_impl::processQtColorizedCloud(f2g::proc pl, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer){

    int errNo = 0;

    bool showFPS = true;

    std::vector<int> iter_ply;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> mCloud;

    f2g::grabber grab(pl, false);       //args:(pipeline, mirror)

    mCloud = grab.createColorizedPointCloud();

    mCloud->sensor_orientation_.w() = 0.0f;
    mCloud->sensor_orientation_.x() = 1.0f;
    mCloud->sensor_orientation_.y() = 0.0f;
    mCloud->sensor_orientation_.z() = 0.0f;

    viewer->setBackgroundColor(0.0f, 0.0f, 0.0f);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(mCloud);

    viewer->setShowFPS(showFPS);
    viewer->addPointCloud<pcl::PointXYZRGB>(mCloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    f2g::saveHelper save(mCloud, false, false, grab);

    viewer->registerKeyboardCallback(f2g::eventlistener::pclSaveEvent, (void*) &save);
    viewer->registerKeyboardCallback(f2g::eventlistener::pclMiscEvent, (void*) &viewer);

    while(!viewer->wasStopped()){
        std::cout<<"viewer loop called"<<std::endl;
        viewer->spinOnce();

        std::chrono::high_resolution_clock::time_point timeNow = std::chrono::high_resolution_clock::now();
        grab.getColorDepthAligned(color_, depth_, mCloud);

        std::chrono::high_resolution_clock::time_point timePost = std::chrono::high_resolution_clock::now();
        std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(timePost-timeNow).count() * 1000 << std::endl;

        viewer->updatePointCloud<pcl::PointXYZRGB> (mCloud, rgb, "sample cloud");
    }

    std::cout<<"processQtColorizedCloud ended"<<std::endl;
    grab.shutdown();

    return(errNo);
}




int f2g::grabber_impl::processQtUncolorizedCloud(f2g::proc pl, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer){
    std::cout<<"process qt uncolorized updated"<<std::endl;

    int errNo = 0;

    bool showFPS = true;

    std::cout<< "Processing Point Cloud..." <<std::endl;

    std::vector<int> iter_ply;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> mCloud;

    f2g::grabber grab(pl, false);       //args:(pipeline, mirror)

    mCloud = grab.createUncolorizedPointCloud();

    mCloud->sensor_orientation_.w() = 0.0f;
    mCloud->sensor_orientation_.x() = 1.0f;
    mCloud->sensor_orientation_.y() = 0.0f;
    mCloud->sensor_orientation_.z() = 0.0f;

    viewer->setBackgroundColor(0.0f, 0.0f, 0.0f);

    viewer->setShowFPS(showFPS);
    viewer->addPointCloud<pcl::PointXYZ>(mCloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    f2g::saveHelper save(mCloud, false, false, grab);

    viewer->registerKeyboardCallback(f2g::eventlistener::pclSaveEvent, (void*) &save);
    viewer->registerKeyboardCallback(f2g::eventlistener::pclMiscEvent, (void*) &viewer);

    while(!viewer->wasStopped()){
        viewer->spinOnce();

        std::chrono::high_resolution_clock::time_point timeNow = std::chrono::high_resolution_clock::now();
        grab.getDepthAligned(depth_, mCloud);

        std::chrono::high_resolution_clock::time_point timePost = std::chrono::high_resolution_clock::now();
        std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(timePost-timeNow).count() * 1000 << std::endl;

        viewer->updatePointCloud<pcl::PointXYZ> (mCloud, "sample cloud");
    }

    grab.shutdown();

    return(errNo);
}




int f2g::grabber_impl::processUncolorizedPointCloud(f2g::proc pl, bool setSize, int xw, int yw){

    int errNo = 0;

    bool showFPS = true;

    std::cout<< "Processing Point Cloud..." <<std::endl;

    std::vector<int> iter_ply;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> mCloud;

    f2g::grabber grab(pl, false);       //args:(pipeline, mirror)

    mCloud = grab.createUncolorizedPointCloud();

    mCloud->sensor_orientation_.w() = 0.0f;
    mCloud->sensor_orientation_.x() = 1.0f;
    mCloud->sensor_orientation_.y() = 0.0f;
    mCloud->sensor_orientation_.z() = 0.0f;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

    viewer->setBackgroundColor(0.0f, 0.0f, 0.0f);

    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(mCloud);

    if(setSize){
        double posx = 0.0;
        double posy = 0.0;
        double posz = -12.0;

        double up_x = 0.0;
        double up_y = 0.0;
        double up_z = 0.0;

        viewer->spinOnce();
        viewer->setSize(xw, yw);
        viewer->setCameraPosition(posx, posy, posz, up_x, up_y, up_z);
    }

    viewer->setShowFPS(showFPS);
    viewer->addPointCloud<pcl::PointXYZ>(mCloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    f2g::saveHelper save(mCloud, false, false, grab);

    viewer->registerKeyboardCallback(f2g::eventlistener::pclSaveEvent, (void*) &save);
    viewer->registerKeyboardCallback(f2g::eventlistener::pclMiscEvent, (void*) &viewer);

    while(!viewer->wasStopped()){
        viewer->spinOnce();

        std::chrono::high_resolution_clock::time_point timeNow = std::chrono::high_resolution_clock::now();
        grab.getDepthAligned(depth_, mCloud);

        /*opencv window block start*/
        //cv::imshow("color", color_);
        //char cv_event =(char) cv::waitKey(10);     //react after 10 msec and cast to char
        //f2g::eventlistener::KeyboardInputEvent(cv_event);
        /*opencv window block end*/

        std::chrono::high_resolution_clock::time_point timePost = std::chrono::high_resolution_clock::now();
        std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(timePost-timeNow).count() * 1000 << std::endl;

        viewer->updatePointCloud<pcl::PointXYZ> (mCloud, "sample cloud");
    }

    grab.shutdown();

    return(errNo);
}


cv::Mat f2g::grabber_impl::getColorMat(){
    return color_;
}

cv::Mat f2g::grabber_impl::getDepthMat(){
    return depth_;
}
