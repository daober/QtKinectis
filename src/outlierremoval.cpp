#include "outlierremoval.hpp"

#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/make_shared.hpp>



processing::outlierremoval::outlierremoval(){

    //allocate new cloud on heap via make shared
    inputCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    outputCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}


int processing::outlierremoval::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out){

    std::cout << "cloud outlierremoval called" <<std::endl;

    reader_.read<pcl::PointXYZ>("filtered_scene_in.pcd", *in);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    sor.setInputCloud(in);

    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*out);

    writer_.write<pcl::PointXYZ>("filtered_inliers.pcd", *out, false);

    return(0);
}
