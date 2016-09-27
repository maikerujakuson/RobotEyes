#ifndef __CAMERA_H_INCLUDED__
#define __CAMERA_H_INCLUDED__

#include<OpenNI.h>
#include<opencv2\opencv.hpp>

class Camera {
public:
	virtual cv::Mat getColorImage(void) = 0;
	virtual cv::Mat getDepthImage(void) = 0;
};

class Kinect : public Camera {
public:
	Kinect();
	virtual cv::Mat getColorImage(void);
	virtual cv::Mat getDepthImage(void);

private:
	openni::Device device;
	int changedIndex = 0;
	int changedIndex2 = 0;
	cv::Mat colorImage;
	cv::Mat depthImage;
	openni::VideoStream colorStream;
	openni::VideoStream depthStream;

	std::vector<openni::VideoStream*> colorStreams;
	std::vector<openni::VideoStream*> depthStreams;
};

#endif