#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/compression/octree_pointcloud_compression.h>

namespace processing{

    class compress{

        public:
        compress();

        int compressData(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud);

        private:

        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud_;  
    };


}
