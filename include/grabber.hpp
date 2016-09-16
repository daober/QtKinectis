/**
*** header for freenect2 to pcl grabber ***

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

#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

#include <iostream>
#include <signal.h>
#include <string.h>
#include <cstdlib>
#include <fstream>



namespace f2g {

	enum proc {CPU, OPENCL, OPENGL, CUDA};

	class grabber {

	public:

		grabber(proc pl = CPU, bool mirror = true, std::string serial = std::string());

		virtual libfreenect2::Freenect2Device::IrCameraParams getIrParameters(void);
		virtual libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters(void);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud(void);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud(const libfreenect2::Frame *rgb, const libfreenect2::Frame *depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(const libfreenect2::Frame * rgb, const libfreenect2::Frame * depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

		void getColorDepthAligned(cv::Mat &colormat, cv::Mat &depthmat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const bool hd = true, const bool rmpoints = true);
		void getColorDepthAligned(cv::Mat & colormat, cv::Mat & depthmat, const bool hd = true, const bool rmpoints = false);

		std::pair<cv::Mat, cv::Mat> getDepthRgb(const bool enable_filter = true, libfreenect2::Frame* bigdepth = 0, int* color_depth_map = 0);

		/**
		 * @brief shutdown kinectv2 device and close program
		 * @return true if ok
		 */
		bool shutdown(void);


		/**
		 * @brief initializes kinectv2 device
		 * @return true if ok
		 */
		bool initialize(void);

		void getRgb(cv::Mat &colormat);
		void getDepth(cv::Mat depthmat);
		void getIr(cv::Mat irmat);

		void setRGBViewer(const bool rgbwin = true);
		bool getRGBViewer();

		void setPCLViewer(const bool pclwin = true);
		bool getPCLViewer();

		void setMirror(const bool mirror = true);
		bool getMirror();

		void setProcessingPipeline(proc pl);
		std::string getProcessingPipeline(void);

		void create3Dcloud(const libfreenect2::Freenect2Device::IrCameraParams &depthPoints);


		/**
		 * @brief enumerates kinectv2 device
		 * @return true if ok
		 */
		bool enumKinectv2Device(void);

		libfreenect2::Freenect2Device *getFreenectDevice(void);
		libfreenect2::SyncMultiFrameListener *getListener(void);

		void printDeviceParams();
		void storeDeviceParams();

	private:

		/*private member vars*/
		libfreenect2::Freenect2 freenect2_;

		libfreenect2::Freenect2Device * dev_ = 0;
		libfreenect2::PacketPipeline * pipeline_ = 0;
		libfreenect2::Registration * registration_ = 0;

		libfreenect2::SyncMultiFrameListener multilistener_;
		libfreenect2::FrameMap frameMap_;

		libfreenect2::Frame undistorted_;
		libfreenect2::Frame registered_;
		libfreenect2::Frame mat_;

		Eigen::Matrix<float,512,1> cols;
		Eigen::Matrix<float,424,1> rows;

		bool createRGBWindow_;
		bool createPCLWindow_;
		bool mirror_;

		std::string serial_;
		std::string procpipe_;

		int map_[512 * 424];

		float qnan_;

    };

}
