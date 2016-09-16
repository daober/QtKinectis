/**
*** source (definition of f2g) for freenect2 to pcl grabber ***

Copyright 2016, Daniel Obermaier

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of these GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author Daniel Obermaier
*/

//include own header first
#include "grabber.hpp"
//#include "io_cloud.hpp"
#include "eventlistener.hpp"


f2g::grabber::grabber(proc pl, bool mirror, std::string serial) :    mirror_(mirror),
                                                                     multilistener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth),
                                                                     createRGBWindow_(true),
                                                                     undistorted_(512, 424, 4),
                                                                     registered_(512, 424, 4),
                                                                     mat_(1920, 1082, 4),
                                                                     qnan_(std::numeric_limits<float>::quiet_NaN()) {



    if(!enumKinectv2Device()){
        exit (-1);
    }
    //device enumeration successful
    else{
        serial_ = freenect2_.getDefaultDeviceSerialNumber();

        setProcessingPipeline(pl);

        registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

        //now create a 3D cloud map
        std::cout<< "creating 3D cloud" << std::endl;
        create3Dcloud(dev_->getIrCameraParams());
    }
}


/*propably not a useful method*/
libfreenect2::Freenect2Device::IrCameraParams f2g::grabber::getIrParameters(void){
	libfreenect2::Freenect2Device::IrCameraParams irparam = dev_->getIrCameraParams();
	return irparam;
}


/*propably not a useful method*/
libfreenect2::Freenect2Device::ColorCameraParams f2g::grabber::getRgbParameters(void){
	libfreenect2::Freenect2Device::ColorCameraParams rgbparam = dev_->getColorCameraParams();
	return rgbparam;
}



bool f2g::grabber::enumKinectv2Device(void){
    if(!freenect2_.enumerateDevices()){
        std::cerr<< "no kinectv2 found" <<std::endl;
        return false;
    }
    else{
        return true;
    }
}



bool f2g::grabber::shutdown(void){
    bool err1= false;
    bool err2 = false;

    err1 = dev_->stop();
    err2 = dev_->close();

    return (err1 & err2);
}



bool f2g::grabber::initialize(void){
    bool err = false;

    dev_ = freenect2_.openDevice(serial_, pipeline_);
    dev_->setColorFrameListener(&multilistener_);
    dev_->setIrAndDepthFrameListener(&multilistener_);

    err = dev_->start();

    return err;
}



std::string f2g::grabber::getProcessingPipeline(void){
    return procpipe_;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr f2g::grabber::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    multilistener_.waitForNewFrame(frameMap_);
    libfreenect2::Frame *rgb = frameMap_[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frameMap_[libfreenect2::Frame::Depth];

    registration_->apply(rgb, depth, &undistorted_, &registered_, true, &mat_, map_);

    float divider = 1000.0f;
    float minBound = 0.0001f;

    const std::size_t sWidth = undistorted_.width;
    const std::size_t sHeight = undistorted_.height;

    /*Hint: CV_8UC4 = 4 Channel, 8 bit unsigned char per channel*/
    cv::Mat tempDepthMat(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
    cv::Mat tempRGBMat(registered_.height, registered_.width, CV_8UC4, registered_.data);

    const float *iterDepth = (float *) tempDepthMat.ptr();
    const char *iterRGB = (char *) tempRGBMat.ptr();

    pcl::PointXYZRGB *iterPoint = &cloud->points[0];

    if (mirror_ == true){
        //cv::flip(inputArray src, outputArray dst, int flipcode (1 = flip around y-axis))
        //TODO: check here later on...
        cv::flip(tempDepthMat, tempDepthMat, 1);
        cv::flip(tempRGBMat, tempRGBMat, 1);
    }

    bool isDense = true;

    for(std::size_t y = 0; y != sHeight; ++y){

        const unsigned int offset = y * sWidth;
        const float *itDepth = iterDepth + offset;
        const char *itRGB = iterRGB + offset * 4;

        const float dy = rows(y);

        for(std::size_t x = 0; x < sWidth; ++x, ++iterPoint, ++itDepth, itRGB += 4){

            const float depthVal = *itDepth / divider;

            if(!std::isnan(depthVal) && !(std::abs(depthVal) < minBound)){

                const float rX = cols(x) * depthVal;
                const float rY = dy * depthVal;

                iterPoint->x = rX;
                iterPoint->y = rY;
                iterPoint->z = depthVal;

                iterPoint->r=itRGB[2];
                iterPoint->g = itRGB[1];
                iterPoint->b = itRGB[0];
            }
            else{
                iterPoint->x = qnan_;
                iterPoint->y = qnan_;
                iterPoint->z = qnan_;

                iterPoint->r = qnan_;
                iterPoint->g = qnan_;
                iterPoint->b = qnan_;

                isDense = false;
            }

        }

    }
    cloud->is_dense = isDense;
    multilistener_.release(frameMap_);

    return cloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr f2g::grabber::updateCloud(const libfreenect2::Frame * rgb, const libfreenect2::Frame * depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

		registration_->apply(rgb, depth, &undistorted_, &registered_, true, &mat_, map_);

		const std::size_t w = undistorted_.width;
		const std::size_t h = undistorted_.height;

        cv::Mat tmpdepth_iter(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
        cv::Mat tmpcolor_iter(registered_.height, registered_.width, CV_8UC4, registered_.data);

        const float * itdepth = (float *) tmpdepth_iter.ptr();
        const char * itcolor = (char *) tmpcolor_iter.ptr();

        if (mirror_){
            cv::flip(tmpdepth_iter, tmpdepth_iter, 1);
            cv::flip(tmpcolor_iter, tmpcolor_iter, 1);
        }

		pcl::PointXYZRGB * itP = &cloud->points[0];

        bool is_dense = true;

		for(std::size_t y = 0; y < h; ++y){

			const unsigned int offset = y * w;
			const float * itD = itdepth + offset;
			const char * itRGB = itcolor + offset * 4;
			const float dy = rows(y);

			for(std::size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4){
				const float depth_value = *itD / 1000.0f;

				if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){

					const float rx = cols(x) * depth_value;
                	const float ry = dy * depth_value;
					itP->z = depth_value;
					itP->x = rx;
					itP->y = ry;

					itP->b = itRGB[0];
					itP->g = itRGB[1];
					itP->r = itRGB[2];
				} else {
					itP->z = qnan_;
					itP->x = qnan_;
					itP->y = qnan_;

					itP->b = qnan_;
					itP->g = qnan_;
					itP->r = qnan_;
					is_dense = false;
 				}
			}
		}

		cloud->is_dense = is_dense;

		return cloud;
}


void f2g::grabber::create3Dcloud(const libfreenect2::Freenect2Device::IrCameraParams &depthPoints){
    const int width = 512;
    const int height = 424;

    float *pColMap = cols.data();
    float *pRowMap = rows.data();

    for(int i = 0; i < width; i++){
        *pColMap++ = (i - depthPoints.cx + 0.5) / depthPoints.fx;
    }
    for(int e = 0; e < height; e++){
        *pRowMap++ = (e - depthPoints.cy + 0.5) / depthPoints.fy;
    }
}



void f2g::grabber::getColorDepthAligned(cv::Mat &colormat, cv::Mat &depthmat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const bool hd, const bool rmpoints){

    multilistener_.waitForNewFrame(frameMap_);

    libfreenect2::Frame *rgb = frameMap_[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frameMap_[libfreenect2::Frame::Depth];

    registration_->apply(rgb, depth, &undistorted_, &registered_, rmpoints, &mat_, map_);

    cv::Mat tmpDepth(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
    cv::Mat tmpColor;

    if(hd){
        tmpColor = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
    }
    else{
        tmpColor = cv::Mat(registered_.height, registered_.width, CV_8UC4, registered_.data);
    }

    depthmat = tmpDepth.clone();
    colormat = tmpColor.clone();

    cloud = getColorizedPointCloud(rgb, depth, cloud);
    multilistener_.release(frameMap_);
}


// Depth and color are aligned and registered
void f2g::grabber::getColorDepthAligned(cv::Mat &colormat, cv::Mat & depthmat, const bool hd, const bool rmpoints){
		multilistener_.waitForNewFrame(frameMap_);
		libfreenect2::Frame * rgb = frameMap_[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frameMap_[libfreenect2::Frame::Depth];

		registration_->apply(rgb, depth, &undistorted_, &registered_, rmpoints, &mat_, map_);

		cv::Mat tmpDepth(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
		cv::Mat tmpColor;

		if(hd){
			tmpColor = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
        }
        else{
			tmpColor = cv::Mat(registered_.height, registered_.width, CV_8UC4, registered_.data);
        }

        if (mirror_ == true){
            cv::flip(tmpDepth, tmpDepth, 1);
            cv::flip(tmpColor, tmpColor, 1);
        }

        colormat = tmpColor.clone();
        depthmat = tmpDepth.clone();

		multilistener_.release(frameMap_);
}



libfreenect2::Freenect2Device *(f2g::grabber::getFreenectDevice(void)){
    return (dev_);
}


libfreenect2::SyncMultiFrameListener *(f2g::grabber::getListener(void)){
    return &(multilistener_);
}


void f2g::grabber::setProcessingPipeline(proc pl){

switch(pl){
    case CPU:
        std::cout << "creating CPU processor" << std::endl;

        if (serial_.empty()){
            dev_ = freenect2_.openDefaultDevice (new libfreenect2::CpuPacketPipeline());
        }
        else{
            dev_ = freenect2_.openDevice (serial_, new libfreenect2::CpuPacketPipeline());
        }
        std::cout << "created" << std::endl;
        break;

    case OPENCL:
        std::cout << "creating OpenCL processor" << std::endl;

        if(serial_.empty()){
            dev_ = freenect2_.openDefaultDevice(new libfreenect2::OpenCLPacketPipeline());
        }
        else{
            dev_ = freenect2_.openDevice(serial_, new libfreenect2::OpenCLPacketPipeline());
        }
        std::cout << "created" << std::endl;
        break;

    case OPENGL:
        std::cout << "creating OpenGL processor" << std::endl;

        if (serial_.empty()){
            dev_ = freenect2_.openDefaultDevice (new libfreenect2::OpenGLPacketPipeline ());
        }
        else{
            dev_ = freenect2_.openDevice (serial_, new libfreenect2::OpenGLPacketPipeline ());
        }
        std::cout << "created" << std::endl;
        break;

    default:
        std::cout << "creating Cpu processor" << std::endl;

        if (serial_.empty()){
            dev_ = freenect2_.openDefaultDevice (new libfreenect2::CpuPacketPipeline ());
        }
        else{
            dev_ = freenect2_.openDevice (serial_, new libfreenect2::CpuPacketPipeline ());
        }
        std::cout << "created" << std::endl;
        break;
}

    serial_ = freenect2_.getDefaultDeviceSerialNumber();

    dev_->setColorFrameListener(&multilistener_);
    dev_->setIrAndDepthFrameListener(&multilistener_);
    dev_->start();

    registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

create3Dcloud(dev_->getIrCameraParams());
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr f2g::grabber::getColorizedPointCloud(void){

    const short width = undistorted_.width;
    const short height = undistorted_.height;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(width, height) );

    return updateCloud(cloud);
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr f2g::grabber::getColorizedPointCloud(const libfreenect2::Frame *rgb, const libfreenect2::Frame *depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    const short width = undistorted_.width;
    const short height = undistorted_.height;

    if(cloud->size() != width * height){
        cloud->resize(width * height);
    }

return updateCloud(rgb, depth, cloud);
}


void f2g::grabber::printDeviceParams(){

    libfreenect2::Freenect2Device::ColorCameraParams colorparams = getRgbParameters();
    std::cout << "rgb fx=" << colorparams.fx << ",fy=" << colorparams.fy <<
        ",cx=" << colorparams.cx << ",cy=" << colorparams.cy << std::endl;

    libfreenect2::Freenect2Device::IrCameraParams irparams = getIrParameters();
    std::cout << "ir fx=" << irparams.fx << ",fy=" << irparams.fy <<
        ",cx=" << irparams.cx << ",cy=" << irparams.cy <<
        ",k1=" << irparams.k1 << ",k2=" << irparams.k2 << ",k3=" << irparams.k3 <<
        ",p1=" << irparams.p1 << ",p2=" << irparams.p2 << std::endl;

}


void f2g::grabber::storeDeviceParams(){

    libfreenect2::Freenect2Device::ColorCameraParams colorparams = getRgbParameters();
    libfreenect2::Freenect2Device::IrCameraParams irparams = getIrParameters();

    cv::Mat rgbmat = (cv::Mat_<float>(3,3) << colorparams.fx, 0, colorparams.cx, 0, colorparams.fy, colorparams.cy, 0, 0, 1);
    cv::Mat depthmat = (cv::Mat_<float>(3,3) << irparams.fx, 0, irparams.cx, 0, irparams.fy, irparams.cy, 0, 0, 1);

    cv::Mat depthdistorted = (cv::Mat_<float>(1,5) << irparams.k1, irparams.k2, irparams.p1, irparams.p2, irparams.k3);

    std::cout<< "storing " << serial_ << std::endl;

    cv::FileStorage fs("calibration_" + serial_ + ".yml", cv::FileStorage::WRITE);

    fs << "CcameraMatrix" << rgbmat;
    fs << "DcameraMatrix" << depthmat << "distorted Coefficients" << depthdistorted;

    fs.release();
}

void f2g::grabber::setMirror(const bool mirror){
    mirror_ = mirror;
}

bool f2g::grabber::getMirror(){
    return mirror_;
}

void f2g::grabber::setRGBViewer(const bool rgbwin){
    createRGBWindow_ = rgbwin;
}

bool f2g::grabber::getRGBViewer(){
    return createRGBWindow_;
}

void f2g::grabber::setPCLViewer(const bool pclwin){
    createPCLWindow_ = pclwin;
}

bool f2g::grabber::getPCLViewer(){
    return createPCLWindow_;
}
