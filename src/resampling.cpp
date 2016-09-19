#include "resampling.hpp"

#include <boost/make_shared.hpp>


int processing::resampling::resampleXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out){

    pcl::io::loadPCDFile("bun0.pcd", *in);

    treeXYZ_ = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointNormal> mls_points;

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    mls.setInputCloud(in);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(treeXYZ_);
    mls.setSearchRadius(0.03);

    mls.process(mls_points);

    pcl::io::savePCDFile("bun0-mls.pcd", mls_points);

}



int processing::resampling::resampleXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out){

    pcl::io::loadPCDFile("bun0.pcd", *in);

    treeXYZRGB_ = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointNormal> mls_points;

    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    mls.setInputCloud(in);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(treeXYZRGB_);
    mls.setSearchRadius(0.03);

    mls.process(mls_points);

    pcl::io::savePCDFile("bun0-mls.pcd", mls_points);
}
