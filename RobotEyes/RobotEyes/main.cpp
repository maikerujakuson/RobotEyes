#include <iostream>
#include <vector>

#define MEASURE_FUNCTION_TIME

#include <pcl\common\time.h>
#include <pcl\common\angles.h>
#include <pcl\console\print.h>
#include <pcl\console\parse.h>
#include <pcl\console\time.h>

#include <pcl\io\pcd_io.h>
#include <pcl\io\openni2_grabber.h>
#include <pcl\io\openni2\openni.h>

#include <pcl\sample_consensus\method_types.h>
#include <pcl\sample_consensus\model_types.h>
#include <pcl\segmentation\sac_segmentation.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\visualization\image_viewer.h>
#include <pcl\visualization\boost.h>
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\point_types.h>

#include <boost\function.hpp>
#include <boost\function_equal.hpp>
#include <boost\bind.hpp>
#include <boost\chrono.hpp>

#include <OpenNI.h>

#include <opencv2\opencv.hpp>

#include "camera.h"
#include "opticalFlow.h"


void segmentation(pcl::PointCloud<pcl::PointXYZRGBA>& cloud, double threshold) {
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointCloud<pcl::PointXYZRGBA>> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(threshold);
	seg.setInputCloud(cloud.makeShared());
	seg.segment(*inliers, *coefficients);

	for (size_t i = 0; i < inliers->indices.size(); ++i) {
		cloud.points[inliers->indices[i]].r = 255;
		cloud.points[inliers->indices[i]].g = 0;
		cloud.points[inliers->indices[i]].b = 0;
	}
}

typedef boost::chrono::high_resolution_clock HRClock;

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
  do \
{ \
  static unsigned count = 0;\
  static double last = pcl::getTime ();\
  double now = pcl::getTime (); \
  ++count; \
  if (now - last >= 1.0) \
{ \
  std::cout << "Average framerate ("<< _WHAT_ << "): " << double (count)/double (now - last) << " Hz" <<  std::endl; \
  count = 0; \
  last = now; \
} \
}while (false)
#else
#define FPS_CALC (_WHAT_) \
  do \
{ \
}while (false)
#endif

void
printHelp(int, char **argv)
{
	using pcl::console::print_error;
	using pcl::console::print_info;

	print_error("Syntax is: %s [((<device_id> | <path-to-oni-file>) [-depthmode <mode>] [-imagemode <mode>] [-xyz] | -l [<device_id>]| -h | --help)]\n", argv[0]);
	print_info("%s -h | --help : shows this help\n", argv[0]);
	print_info("%s -xyz : use only XYZ values and ignore RGB components (this flag is required for use with ASUS Xtion Pro) \n", argv[0]);
	print_info("%s -l : list all available devices\n", argv[0]);
	print_info("%s -l <device-id> :list all available modes for specified device\n", argv[0]);
	print_info("\t\t<device_id> may be \"#1\", \"#2\", ... for the first, second etc device in the list\n");
#ifndef _WIN32
	print_info("\t\t                   bus@address for the device connected to a specific usb-bus / address combination\n");
	print_info("\t\t                   <serial-number>\n");
#endif
	print_info("\n\nexamples:\n");
	print_info("%s \"#1\"\n", argv[0]);
	print_info("\t\t uses the first device.\n");
	print_info("%s  \"./temp/test.oni\"\n", argv[0]);
	print_info("\t\t uses the oni-player device to play back oni file given by path.\n");
	print_info("%s -l\n", argv[0]);
	print_info("\t\t list all available devices.\n");
	print_info("%s -l \"#2\"\n", argv[0]);
	print_info("\t\t list all available modes for the second device.\n");
#ifndef _WIN32
	print_info("%s A00361800903049A\n", argv[0]);
	print_info("\t\t uses the device with the serial number \'A00361800903049A\'.\n");
	print_info("%s 1@16\n", argv[0]);
	print_info("\t\t uses the device on address 16 at USB bus 1.\n");
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Create the PCLVisualizer object
boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
boost::shared_ptr<pcl::visualization::ImageViewer> img;

/* ---[ */
int
main(int argc, char** argv)
{
	std::string device_id("");
	pcl::io::OpenNI2Grabber::Mode depth_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
	pcl::io::OpenNI2Grabber::Mode image_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
	bool xyz = false;

	if (argc >= 2)
	{
		device_id = argv[1];
		if (device_id == "--help" || device_id == "-h")
		{
			printHelp(argc, argv);
			return 0;
		}
		else if (device_id == "-l")
		{
			if (argc >= 3)
			{
				pcl::io::OpenNI2Grabber grabber(argv[2]);
				boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = grabber.getDevice();
				cout << *device;		// Prints out all sensor data, including supported video modes
			}
			else
			{
				boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance();
				if (deviceManager->getNumOfConnectedDevices() > 0)
				{
					for (unsigned deviceIdx = 0; deviceIdx < deviceManager->getNumOfConnectedDevices(); ++deviceIdx)
					{
						boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = deviceManager->getDeviceByIndex(deviceIdx);
						cout << "Device " << device->getStringID() << "connected." << endl;
					}

				}
				else
					cout << "No devices connected." << endl;

				cout << "Virtual Devices available: ONI player" << endl;
			}
			return 0;
		}
	}
	else
	{
		boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance();
		if (deviceManager->getNumOfConnectedDevices() > 0)
		{
			boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = deviceManager->getAnyDevice();
			cout << "Device ID not set, using default device: " << device->getStringID() << endl;
		}
	}

	unsigned mode;
	if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
		depth_mode = pcl::io::OpenNI2Grabber::Mode(mode);

	if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
		image_mode = pcl::io::OpenNI2Grabber::Mode(mode);

	if (pcl::console::find_argument(argc, argv, "-xyz") != -1)
		xyz = true;

	pcl::io::OpenNI2Grabber grabber(device_id, depth_mode, image_mode);

	if (xyz || !grabber.providesCallback<pcl::io::OpenNI2Grabber::sig_cb_openni_point_cloud_rgb>())
	{
		OpenNI2Viewer<pcl::PointXYZ> openni_viewer(grabber);
		openni_viewer.run();
	}
	else
	{
		OpenNI2Viewer<pcl::PointXYZRGBA> openni_viewer(grabber);
		openni_viewer.run();
	}

	return (0);
}