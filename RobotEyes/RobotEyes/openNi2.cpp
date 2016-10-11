#include "openNi2.h"

template <typename PointType>
void OpenNI2Viewer::cloud_callback(const OpenNI2Viewer::CloudConstPtr& cloud)
{
	boost::mutex::scoped_lock lock(cloud_mutex_);
	cloud_ = cloud;
	pcl::PointCloud<OpenNI2Viewer::Cloud> segmented_cloud(*cloud);
}

template <typename PointType>
void OpenNI2Viewer::image_callback(const boost::shared_ptr<pcl::io::openni2::Image>& image)
{
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

template <typename PointType>
void OpenNI2Viewer::keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
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

template <typename PointType>
void OpenNI2Viewer::mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void*)
{
	if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
	{
		cout << "left button pressed @ " << mouse_event.getX() << " , " << mouse_event.getY() << endl;
	}
}

template <typename PointType>
void OpenNI2Viewer::run()
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
