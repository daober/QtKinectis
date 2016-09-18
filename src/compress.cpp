
//include own header
#include "compress.hpp"

#include <boost/make_shared.hpp>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <iostream>


processing::compress::compress(){

    inputCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
}

int processing::compress::compressData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud){

    bool showStatistics = true;

    pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
    // instantiate point cloud compression for encoding and decoding
    PointCloudEncoder = boost::make_shared<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>>(compressionProfile, showStatistics);

    PointCloudEncoder->encodePointCloud(inCloud, compressedData);

    return (0);
}
