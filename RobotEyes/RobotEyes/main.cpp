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

#include "camera.h"
#include "opticalFlow.h"
#include "frame_process.hpp"

// Create the PCLVisualizer object
boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
boost::shared_ptr<pcl::visualization::ImageViewer> img;

int
main(int argc, char** argv)
{
	std::string device_id("");
	pcl::io::OpenNI2Grabber::Mode depth_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
	pcl::io::OpenNI2Grabber::Mode image_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;

	unsigned mode;
	if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
		depth_mode = pcl::io::OpenNI2Grabber::Mode(mode);

	if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
		image_mode = pcl::io::OpenNI2Grabber::Mode(mode);

	pcl::io::OpenNI2Grabber grabber(device_id, depth_mode, image_mode);

	FrameProcess openni_viewer(grabber);
	openni_viewer.run();

	//cv::Mat depth, color;
	//Kinect kinect;

	//while (true) {
	//	kinect.adjustDepthToColor();
	//}

	return (0);
}