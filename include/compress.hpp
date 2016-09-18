#pragma once

#include <pcl/compression/octree_pointcloud_compression.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


namespace processing{

    class compress{

        public:
        compress();

        int compressData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud);

        private:

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud_;

        boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>> PointCloudEncoder;
        boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>> PointCloudDecoder;

        std::stringstream compressedData;
    };

}
