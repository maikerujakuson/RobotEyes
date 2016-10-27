#include <iostream>
#include <vector>

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <boost/format.hpp>
#include <pcl\tracking\tracking.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//
//#include "camera.h"
//#include "opticalFlow.h"
//#include "frame_process.hpp"
//
//// Create the PCLVisualizer object
//boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
//boost::shared_ptr<pcl::visualization::ImageViewer> img;
//
//int
//main(int argc, char** argv)
//{
//	std::string device_id("");
//	pcl::io::OpenNI2Grabber::Mode depth_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
//	pcl::io::OpenNI2Grabber::Mode image_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
//
//	unsigned mode;
//	if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
//		depth_mode = pcl::io::OpenNI2Grabber::Mode(mode);
//
//	if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
//		image_mode = pcl::io::OpenNI2Grabber::Mode(mode);
//
//	pcl::io::OpenNI2Grabber grabber(device_id, depth_mode, image_mode);
//
//	FrameProcess openni_viewer(grabber);
//	openni_viewer.run();
//
//	//cv::Mat depth, color;
//	//Kinect kinect;
//
//	//while (true) {
//	//	kinect.adjustDepthToColor();
//	//}
//
//	return (0);
//}
/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* $Id$
*
*/

#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <boost/chrono.hpp>

#include "pcl/io/openni2/openni.h"

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


// Function to print program usage
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

// Class to manage openni2grabber
template <typename PointType>
class OpenNI2Viewer
{
public:
	// Use template point type 
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::ConstPtr CloudConstPtr;

	// Constructer 
	OpenNI2Viewer(pcl::io::OpenNI2Grabber& grabber)
		: cloud_viewer_(new pcl::visualization::PCLVisualizer("PCL OpenNI2 cloud"))
		, image_viewer_()
		, grabber_(grabber)
		, rgb_data_(0), rgb_data_size_(0)
	{
	}

	// Point cloud callback function
	// This function is called when pointcloud is updated
	void
		cloud_callback(const CloudConstPtr& cloud)
	{
		// Calculate the FPS of point cloud viewer
		FPS_CALC("cloud callback");
		// Lock cloud_mutex
		boost::mutex::scoped_lock lock(cloud_mutex_);
		// Set updated point cloud to member point cloud
		//cloud_ = cloud;
	
		Cloud::Ptr cloud2(new Cloud(*cloud)), cloud_f(new Cloud);

		// Filter point cloud by distance
		pcl::PassThrough<PointType> ptfilter(true);
		ptfilter.setInputCloud(cloud);
		// Cut x dimention between -50cm and 50cm
		ptfilter.setFilterFieldName("x");
		ptfilter.setFilterLimits(-0.5f, 0.5f);
		ptfilter.filter(*cloud2);
		ptfilter.setInputCloud(cloud2);
		ptfilter.setFilterFieldName("z");
		ptfilter.setFilterLimits(0.0f, 4.0f);
		ptfilter.filter(*cloud_f);

		// Find points belonging to plane 
		pcl::SACSegmentation<PointType> seg;
		seg.setInputCloud(cloud_f);
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.02f);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		seg.segment(*inliers, *coefficients);

		// Extract points except for plane
		pcl::ExtractIndices<PointType> extract(true);
		extract.setInputCloud(cloud_f);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud2);

		// Cluster point cloud
		pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
		tree->setInputCloud(cloud2);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointType> ec;
		// Set distance threshold 2cm
		ec.setClusterTolerance(0.02f);
		ec.setMinClusterSize(1000);
		ec.setMaxClusterSize(300000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud2);
		ec.extract(cluster_indices);

		extract.setInputCloud(cloud2);
		pcl::IndicesPtr indices(new std::vector<int>);
		*indices = cluster_indices[0].indices;
		extract.setIndices(indices);
		extract.setNegative(false);
		extract.filter(*cloud_f);

		// Set proccesed point cloud to viewer
		cloud_ = cloud_f;
		

	}
	
	// Image callback function
	// This function is called when image is updated
	void
		image_callback(const boost::shared_ptr<pcl::io::openni2::Image>& image)
	{
		// Calculate the FPS of image viewer
		FPS_CALC("image callback");
		// Lock image_mutex
		boost::mutex::scoped_lock lock(image_mutex_);
		// Set updated image to member image
		image_ = image;

		// Cheack if updated image is suited to pcl image viewer 
		if (image->getEncoding() != pcl::io::openni2::Image::RGB)
		{
			// Make suited image 
			if (rgb_data_size_ < image->getWidth() * image->getHeight())
			{
				if (rgb_data_)
					delete[] rgb_data_;
				rgb_data_size_ = image->getWidth() * image->getHeight();
				rgb_data_ = new unsigned char[rgb_data_size_ * 3];
			}
			image_->fillRGB(image_->getWidth(), image_->getHeight(), rgb_data_);
		}
	}

	// Keyboard callback function
	void
		keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
	{
		if (event.getKeyCode())
			cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
		else
			cout << "the special key \'" << event.getKeySym() << "\' was";
		if (event.keyDown())
			cout << " pressed" << endl;
		else
			cout << " released" << endl;
	}
	
	// Mouse callback function
	void
		mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void*)
	{
		if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
		{
			cout << "left button pressed @ " << mouse_event.getX() << " , " << mouse_event.getY() << endl;
		}
	}

	// This function acts as main loop
	void
		run()
	{
		// Register mouse callback function with openni2viewer
		cloud_viewer_->registerMouseCallback(&OpenNI2Viewer::mouse_callback, *this);
		// Register keyboard callback function with openni2viewer
		cloud_viewer_->registerKeyboardCallback(&OpenNI2Viewer::keyboard_callback, *this);
		// Set FOV of camera 
		cloud_viewer_->setCameraFieldOfView(1.02259994f);
		// Register point cloud callback function with openni2viewr using boost
		boost::function<void(const CloudConstPtr&) > cloud_cb = boost::bind(&OpenNI2Viewer::cloud_callback, this, _1);
		boost::signals2::connection cloud_connection = grabber_.registerCallback(cloud_cb);

		// Register image callback function with openni2viewer using boost
		boost::signals2::connection image_connection;
		if (grabber_.providesCallback<void(const boost::shared_ptr<pcl::io::openni2::Image>&)>())
		{
			image_viewer_.reset(new pcl::visualization::ImageViewer("PCL OpenNI image"));
			image_viewer_->registerMouseCallback(&OpenNI2Viewer::mouse_callback, *this);
			image_viewer_->registerKeyboardCallback(&OpenNI2Viewer::keyboard_callback, *this);
			boost::function<void(const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb = boost::bind(&OpenNI2Viewer::image_callback, this, _1);
			image_connection = grabber_.registerCallback(image_cb);
		}

		bool image_init = false, cloud_init = false;

		grabber_.start();

		// Do loop untile viewer windows is terminated
		while (!cloud_viewer_->wasStopped() && (image_viewer_ && !image_viewer_->wasStopped()))
		{
			boost::shared_ptr<pcl::io::openni2::Image> image;
			CloudConstPtr cloud;

			cloud_viewer_->spinOnce();

			// See if we can get a cloud
			if (cloud_mutex_.try_lock())
			{
				cloud_.swap(cloud);
				cloud_mutex_.unlock();
			}

			if (cloud)
			{
				FPS_CALC("drawing cloud");

				if (!cloud_init)
				{
					cloud_viewer_->setPosition(0, 0);
					cloud_viewer_->setSize(cloud->width, cloud->height);
					cloud_init = !cloud_init;
				}

				if (!cloud_viewer_->updatePointCloud(cloud, "OpenNICloud"))
				{
					cloud_viewer_->addPointCloud(cloud, "OpenNICloud");
					cloud_viewer_->resetCameraViewpoint("OpenNICloud");
					cloud_viewer_->setCameraPosition(
						0, 0, 0,		// Position
						0, 0, 1,		// Viewpoint
						0, -1, 0);	// Up
				}
			}

			// See if we can get an image
			if (image_mutex_.try_lock())
			{
				image_.swap(image);
				image_mutex_.unlock();
			}


			if (image)
			{
				if (!image_init && cloud && cloud->width != 0)
				{
					image_viewer_->setPosition(cloud->width, 0);
					image_viewer_->setSize(cloud->width, cloud->height);
					image_init = !image_init;
				}

				if (image->getEncoding() == pcl::io::openni2::Image::RGB)
					image_viewer_->addRGBImage((const unsigned char*)image->getData(), image->getWidth(), image->getHeight());
				else
					image_viewer_->addRGBImage(rgb_data_, image->getWidth(), image->getHeight());
				image_viewer_->spinOnce();

			}
		}

		grabber_.stop();

		cloud_connection.disconnect();
		image_connection.disconnect();
		if (rgb_data_)
			delete[] rgb_data_;
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
	boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;

	pcl::io::OpenNI2Grabber& grabber_;
	// Mutex object to protect point cloud ownership 
	boost::mutex cloud_mutex_;
	// Mutex object to protect RGB image onwership
	boost::mutex image_mutex_;

	CloudConstPtr cloud_;
	boost::shared_ptr<pcl::io::openni2::Image> image_;
	unsigned char* rgb_data_;
	unsigned rgb_data_size_;
};

// Shared pointer for PCLVisualizer (Pointcloud) 
boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
// Share pointer for ImageViewer (RGB image)
boost::shared_ptr<pcl::visualization::ImageViewer> img;

// MAIN FUNCTION
int
main(int argc, char** argv)
{
	// String Variable for camera device ID
	std::string device_id("");
	
	// 
	pcl::io::OpenNI2Grabber::Mode depth_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
	pcl::io::OpenNI2Grabber::Mode image_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
	bool xyz = false;

	// Check if the number of arguments is accurate
	if (argc >= 2)
	{
		// When the number of arguments is accurate

		// Get commands argument
		device_id = argv[1];

		// Cheack if a command is help or not 
		if (device_id == "--help" || device_id == "-h")
		{
			// Print program argument usage 
			printHelp(argc, argv);
			return 0;
		}
		// When argument is "-l", list all availabe camera device
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
				// Shared pointer to store an adress of device manager
				boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance();
				// Cheack if any devices are availabe
				if (deviceManager->getNumOfConnectedDevices() > 0)
				{
					// Print all available devices
					for (unsigned deviceIdx = 0; deviceIdx < deviceManager->getNumOfConnectedDevices(); ++deviceIdx)
					{
						boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = deviceManager->getDeviceByIndex(deviceIdx);
						cout << "Device " << device->getStringID() << "connected." << endl;
					}

				}
				// There is no availabe device
				else
					cout << "No devices connected." << endl;
			}
			return 0;
		}
	}
	//	When a device ID is not indicated
	else
	{
		// Tell that program will use default device
		boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance();
		if (deviceManager->getNumOfConnectedDevices() > 0)
		{
			boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = deviceManager->getAnyDevice();
			cout << "Device ID not set, using default device: " << device->getStringID() << endl;
		}
	}

	// To do: When "-depthmode" or "-imagemode" is used bug occures 
	unsigned mode;
	if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
		depth_mode = pcl::io::OpenNI2Grabber::Mode(mode);

	if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
		image_mode = pcl::io::OpenNI2Grabber::Mode(mode);

	// Use "-xyz" mode
	if (pcl::console::find_argument(argc, argv, "-xyz") != -1)
		xyz = true;

	// Accuire openni2grabber object
	pcl::io::OpenNI2Grabber grabber(device_id, depth_mode, image_mode);

	// Cheack if "-xyz" mode is used
	if (xyz || !grabber.providesCallback<pcl::io::OpenNI2Grabber::sig_cb_openni_point_cloud_rgb>())
	{
		// Use white color point cloud
		OpenNI2Viewer<pcl::PointXYZ> openni_viewer(grabber);
		openni_viewer.run();
	}
	else
	{
		// Use colored point cloud
		OpenNI2Viewer<pcl::PointXYZRGBA> openni_viewer(grabber);
		openni_viewer.run();
	}

	return (0);
}