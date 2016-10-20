#include "camera.h"
#include <pcl\point_types.h>
#include <pcl\io\pcd_io.h>
#include <pcl\io\io.h>
#include <pcl\visualization\cloud_viewer.h>

Kinect::Kinect() {
	openni::OpenNI::initialize();

	if (device.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
		//return -1;
	}
	// Depth Ç∆ Color ÇÃÉtÉåÅ[ÉÄÇìØä˙Ç≥ÇπÇÈ
	device.setDepthColorSyncEnabled(true);
	//:w
	device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

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

void Kinect::adjustDepthToColor()
{
	int x, y;
	int colorX, colorY;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::io::loadPCDFile("raw_0.pcd", *point_cloud_ptr);
	openni::VideoFrameRef depthFrame;
	depthStream.readFrame(&depthFrame);
	cv::Mat color, depth;
	depth = this->getDepthImage();
	color = this->getColorImage();
		float X, Y, Z;
		

		//viewer.showCloud(point_cloud_ptr);
		while (!viewer.wasStopped())
		{
			depth = this->getDepthImage();
			color = this->getColorImage();
			point_cloud_ptr->points.clear();
			for (int i = 0; i < depth.rows; i++)
			{
				for (int j = 0; j < depth.cols; j++)
				{

					openni::CoordinateConverter::convertDepthToWorld(this->depthStream, i, j, (openni::DepthPixel)depth.at<unsigned short>(i, j), &X, &Y, &Z);
					pcl::PointXYZRGB point;
					point.x = X;
					point.y = Y;
					point.z = Z;
					openni::CoordinateConverter::convertDepthToColor(this->depthStream, this->colorStream,i,  j, (openni::DepthPixel)depth.at<unsigned short>(i, j), &colorX, &colorY);
					point.r = color.at<cv::Vec3b>(colorX, colorY)[2];
					point.g = color.at<cv::Vec3b>(colorX, colorY)[1];
					point.b = color.at<cv::Vec3b>(colorX, colorY)[0];
					point_cloud_ptr->points.push_back(point);
				}
			}
			//cout << "X:" << X << endl;
			point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
			point_cloud_ptr->height = 1;
			//cout << point_cloud_ptr->at(0, 0) << endl;
			viewer.showCloud(point_cloud_ptr);
			cv::circle(depth, cv::Point(300, 200), 10, cv::Scalar(0, 0, 255), -1, 8);
			openni::CoordinateConverter::convertDepthToColor(this->depthStream, this->colorStream, 300, 200, (openni::DepthPixel)depth.at<unsigned short>(300,200), &x, &y);
			cv::circle(color, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), -1, 8);
			cv::imshow("color image", color);
			cv::imshow("depth image", depth);
			cv::waitKey(30);
	}



}
