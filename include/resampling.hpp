#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>


namespace processing{

    class resampling{

    public:

    int resampleXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud);
    int resampleXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud);

    private:    

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeXYZ_;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeXYZRGB_;

    };

}
