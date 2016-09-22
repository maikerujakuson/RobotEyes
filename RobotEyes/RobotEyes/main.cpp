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

#include"camera.h"

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


int main()
{
	std::cout << "Hello Omanman" << std::endl;

	//SimpleOpenNIViewer v;
	//v.run();

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