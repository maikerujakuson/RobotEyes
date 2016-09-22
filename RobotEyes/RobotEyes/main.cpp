#include<iostream>
#include<vector>

#include<pcl\io\pcd_io.h>
#include<pcl\io\openni2_grabber.h>
#include<pcl\visualization\cloud_viewer.h>
#include<pcl\point_types.h>

#include<boost\function.hpp>
#include<boost\function_equal.hpp>
#include<boost\bind.hpp>

#include<OpenNI.h>

#include<opencv2\opencv.hpp>

cv::Mat getColorImage(openni::VideoFrameRef& color_frame)
{
	if (!color_frame.isValid())
	{
		return cv::Mat();
	}

	openni::VideoMode video_mode = color_frame.getVideoMode();
	cv::Mat color_img = cv::Mat(video_mode.getResolutionX(),
		video_mode.getResolutionY(), CV_8UC3, (char*)color_frame.getData());
	cv::Mat ret_img;
	cv::cvtColor(color_img, ret_img, CV_RGB2BGR);
	return ret_img;
}

// CV_16U
cv::Mat getDepthImage(openni::VideoFrameRef& depth_frame)
{
	if (!depth_frame.isValid())
	{
		return cv::Mat();
	}

	openni::VideoMode video_mode = depth_frame.getVideoMode();
	cv::Mat depth_img = cv::Mat(video_mode.getResolutionY(),
		video_mode.getResolutionX(),
		CV_16U, (char*)depth_frame.getData());
	return depth_img.clone();
}

cv::Mat getDepthDrawableImage(cv::Mat depth_image)
{
	cv::Mat drawable;
	depth_image.convertTo(drawable, CV_8UC1, 255.0 / 10000);
	drawable = 255 - drawable; // ‹ß‚¢•û‚ð”’‚É
	return drawable;
}

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer") {}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped())
			viewer.showCloud(cloud);
	}

	void run()
	{
		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

		interface->registerCallback(f);
		interface->start();

		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}

		interface->stop();

	}

	pcl::visualization::CloudViewer viewer;

};

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
	int changedIndex = 0;
	int changedIndex2 = 0;
	cv::Mat colorImage;
	cv::Mat depthImage;
	openni::VideoStream colorStream;
	openni::VideoStream depthStream;

	std::vector<openni::VideoStream*> colorStreams;
	std::vector<openni::VideoStream*> depthStreams;
};

Kinect::Kinect() {
	openni::OpenNI::initialize();
	openni::Device device;
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

int main()
{

	std::cout << "Hello Omanman" << std::endl;

	SimpleOpenNIViewer v;
	v.run();

	Kinect kinect;
	while (1)
	{
		cv::imshow("color Image", kinect.getColorImage());
		cv::imshow("depth Image", kinect.getDepthImage());
		int key = cv::waitKey(10);
		if (key == 'q') {
			break;
		}
	}

	return 0;
}