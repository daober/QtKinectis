#include "compress.hpp"

#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <iostream>


processing::compress::compress(){

    //inputCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    //outputCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

int processing::compress::compressData(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud){

    bool showStatistics = true;

    std::stringstream compressedData;

    //PointCloudEncoder->encodePointCloud(inCloud, outCloud);

return (0);
}
