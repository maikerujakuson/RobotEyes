#include "frame_process.hpp" 

// include file for object tracking
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


// Function to execute user-defined cloud processes in one frame 
void 
FrameProcess::cloud_callback(const FrameProcess::CloudConstPtr& cloud)
{
	// Lock the cloud_mutex until the scoped is exited	
	boost::mutex::scoped_lock lock(cloud_mutex_);
	

	// Set streamed cloud to the member cloud
	cloud_ = cloud;
}

// Function to execute user-defined color image processes at one frame color image 
void 
FrameProcess::image_callback(const boost::shared_ptr<pcl::io::openni2::Image>& image)
{
	// Lock the image_mutex until the scoped is exited  	
	boost::mutex::scoped_lock lock(image_mutex_);
	// Set streamed image to the member image
	image_ = image;

	// Check if the encoding of image is same as the one of Openni2 RGB
	if (image->getEncoding() != pcl::io::openni2::Image::RGB)
	{
		// Use rgb_data insted of image_	
		if (rgb_data_size_ < image->getWidth() * image->getHeight())
		{
			if (rgb_data_)
				delete[] rgb_data_;
			rgb_data_size_ = image->getWidth() * image->getHeight();
			rgb_data_ = new unsigned char[rgb_data_size_ * 3];
		}
		image_->fillRGB(image_->getWidth(), image_->getHeight(), rgb_data_);
	}

	// Set streamed color image to color
	cv::Mat color = cv::Mat(image_->getHeight(), image_->getWidth(), CV_8UC3);
	image_->fillRGB(color.cols, color.rows, color.data, color.step);
	cv::cvtColor(color, color, CV_RGB2BGR);
	cv::imshow("color image", color);
	cv::waitKey(30);
}

// Function to execute processes when receiving keyboard inputs
void 
FrameProcess::keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
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

//	Function to execute processes when mouse events happen 
void 
FrameProcess::mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void*)
{
	if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
	{
		cout << "left button pressed @ " << mouse_event.getX() << " , " << mouse_event.getY() << endl;
	}
}

// 
void 
FrameProcess::run()
{
	cloud_viewer_->registerMouseCallback(&FrameProcess::mouse_callback, *this);
	cloud_viewer_->registerKeyboardCallback(&FrameProcess::keyboard_callback, *this);
	cloud_viewer_->setCameraFieldOfView(1.02259994f);
	boost::function<void(const CloudConstPtr&) > cloud_cb = boost::bind(&FrameProcess::cloud_callback, this, _1);
	boost::signals2::connection cloud_connection = grabber_.registerCallback(cloud_cb);

	boost::signals2::connection image_connection;
	if (grabber_.providesCallback<void(const boost::shared_ptr<pcl::io::openni2::Image>&)>())
	{
		image_viewer_.reset(new pcl::visualization::ImageViewer("PCL OpenNI image"));
		image_viewer_->registerMouseCallback(&FrameProcess::mouse_callback, *this);
		image_viewer_->registerKeyboardCallback(&FrameProcess::keyboard_callback, *this);
		boost::function<void(const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb = boost::bind(&FrameProcess::image_callback, this, _1);
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
