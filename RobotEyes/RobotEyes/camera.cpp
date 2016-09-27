#include"camera.h"

Kinect::Kinect() {
	openni::OpenNI::initialize();

	if (device.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
		//return -1;
	}

	colorStream.create(device, openni::SENSOR_COLOR);
	depthStream.create(device, openni::SENSOR_DEPTH);
	colorStream.start();
	depthStream.start();

	openni::Recorder colorRecorder;
	colorRecorder.create("kinect.oni");
	colorRecorder.attach(colorStream);
	colorRecorder.start();


	openni::Recorder depthRecorder;
	depthRecorder.create("kinect.oni");
	depthRecorder.attach(depthStream);
	depthRecorder.start();

	colorStreams.push_back(&colorStream);
	depthStreams.push_back(&depthStream);
}

cv::Mat Kinect::getColorImage(void) {

	openni::OpenNI::waitForAnyStream(&colorStreams[0], colorStreams.size(), &changedIndex);
	if (changedIndex == 0)
	{
		openni::VideoFrameRef colorFrame;
		colorStream.readFrame(&colorFrame);
		if (colorFrame.isValid())
		{
			colorImage = cv::Mat(colorStream.getVideoMode().getResolutionY(),
				colorStream.getVideoMode().getResolutionX(),
				CV_8UC3, (char*)colorFrame.getData());
			cv::cvtColor(colorImage, colorImage, CV_BGR2RGB);
		}

	}

	return colorImage;
}

cv::Mat Kinect::getDepthImage(void) {

	openni::OpenNI::waitForAnyStream(&depthStreams[0], depthStreams.size(), &changedIndex2);
	if (changedIndex2 == 0)
	{
		openni::VideoFrameRef depthFrame;
		depthStream.readFrame(&depthFrame);
		if (depthFrame.isValid())
		{
			depthImage = cv::Mat(depthStream.getVideoMode().getResolutionY(),
				depthStream.getVideoMode().getResolutionX(),
				CV_16U, (char*)depthFrame.getData());
		}
	}
	return depthImage;
}
