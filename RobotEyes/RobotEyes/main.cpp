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
//
//class SimpleOpenNIViewer
//{
//public:
//	SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer") 
//		, image_viewer(){}
//
//	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
//	{
//		if (!viewer.wasStopped()) {
//			//pcl::PointCloud<pcl::PointXYZRGB> segmented_cloud(*cloud);
//			//segmentation(segmented_cloud, 0.08);
//			viewer.showCloud(cloud);
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
//			*cloud_copy = *cloud;
//			iv.addRGBImage<pcl::PointXYZRGB>(cloud_copy);
//			iv.spin();*/
//			//const boost::shared_ptr<pcl::io::openni2::Image>& rgb1;
//			//rgb1->fillRGB
//
//		}
//	}
//
//	void run()
//	{
//		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
//
//		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
//			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
//		
//
//		interface->registerCallback(f);
//		interface->start();
//		while (!viewer.wasStopped())
//		{
//			boost::this_thread::sleep(boost::posix_time::seconds(1));
//		}
//
//		interface->stop();
//
//	}
//
//	pcl::visualization::CloudViewer viewer;
//	boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer;
//	
//};
//
//
//int main()
//{
//	//Kinect kinect;
//	SimpleOpenNIViewer vi;
//	//Kinect kinect;
//	vi.run();
//	//while (1)
//	//{
//	//	cv::imshow("color Image", kinect.getColorImage());
//	//	cv::imshow("depth Image", kinect.getDepthImage());
//	//	int key = cv::waitKey(10);
//	//	if (key == 'q') {
//	//		break;
//	//	}
//	//}
//
//	return 0;
//}

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
template <typename PointType>
class OpenNI2Viewer
{
public:
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::ConstPtr CloudConstPtr;

	OpenNI2Viewer(pcl::io::OpenNI2Grabber& grabber)
		: cloud_viewer_(new pcl::visualization::PCLVisualizer("PCL OpenNI2 cloud"))
		, image_viewer_()
		, grabber_(grabber)
		, rgb_data_(0), rgb_data_size_(0)
	{
	}

	void
		cloud_callback(const CloudConstPtr& cloud)
	{
		FPS_CALC("cloud callback");
		boost::mutex::scoped_lock lock(cloud_mutex_);
		cloud_ = cloud;
		pcl::PointCloud<Cloud> segmented_cloud(*cloud);
		segmentation(segmented_cloud, 0.08);
		
	}

	void
		image_callback(const boost::shared_ptr<pcl::io::openni2::Image>& image)
	{
		FPS_CALC("image callback");
		boost::mutex::scoped_lock lock(image_mutex_);
		image_ = image;

		if (image->getEncoding() != pcl::io::openni2::Image::RGB)
		{
			if (rgb_data_size_ < image->getWidth() * image->getHeight())
			{
				if (rgb_data_)
					delete[] rgb_data_;
				rgb_data_size_ = image->getWidth() * image->getHeight();
				rgb_data_ = new unsigned char[rgb_data_size_ * 3];
			}
			image_->fillRGB(image_->getWidth(), image_->getHeight(), rgb_data_);
		}

		cv::Mat C1 = cv::Mat(image_->getHeight(), image_->getWidth(), CV_8UC3);
		image_->fillRGB(C1.cols, C1.rows, C1.data, C1.step);
		cv::cvtColor(C1, C1, CV_RGB2BGR);
		cv::imshow("color image", C1);
		cv::waitKey(30);
	}

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

	void
		mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void*)
	{
		if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
		{
			cout << "left button pressed @ " << mouse_event.getX() << " , " << mouse_event.getY() << endl;
		}
	}

	/**
	* @brief starts the main loop
	*/
	void
		run()
	{
		cloud_viewer_->registerMouseCallback(&OpenNI2Viewer::mouse_callback, *this);
		cloud_viewer_->registerKeyboardCallback(&OpenNI2Viewer::keyboard_callback, *this);
		cloud_viewer_->setCameraFieldOfView(1.02259994f);
		boost::function<void(const CloudConstPtr&) > cloud_cb = boost::bind(&OpenNI2Viewer::cloud_callback, this, _1);
		boost::signals2::connection cloud_connection = grabber_.registerCallback(cloud_cb);

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
				//cv::waitKey(0);

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
	boost::mutex cloud_mutex_;
	boost::mutex image_mutex_;

	CloudConstPtr cloud_;
	boost::shared_ptr<pcl::io::openni2::Image> image_;
	unsigned char* rgb_data_;
	unsigned rgb_data_size_;
};

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