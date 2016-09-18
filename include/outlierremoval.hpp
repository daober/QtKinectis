#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


namespace processing{

    class outlierremoval{
    public:

    outlierremoval();

    int filter( pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud);

    pcl::PCDReader reader_;
    pcl::PCDWriter writer_;

    private:    

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud_; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud_; 

    };


}
