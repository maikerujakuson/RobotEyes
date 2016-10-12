#ifndef __FRAME_PROCESS_HPP_INCLUDED__
#define __FRAME_PROCESS_HPP_INCLUDED__

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

class FrameProcess
{
public:
	typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
	typedef typename Cloud::ConstPtr CloudConstPtr;

	FrameProcess(pcl::io::OpenNI2Grabber& grabber)
		: cloud_viewer_(new pcl::visualization::PCLVisualizer("PCL OpenNI2 cloud"))
		, image_viewer_()
		, grabber_(grabber)
		, rgb_data_(0), rgb_data_size_(0)
	{}

	void cloud_callback(const CloudConstPtr& cloud);
	void image_callback(const boost::shared_ptr<pcl::io::openni2::Image>& image);
	void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*);
	void mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void*);
	void run();

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

#endif // !__FRAME_PROCESS_HPP_INCLUDED__
