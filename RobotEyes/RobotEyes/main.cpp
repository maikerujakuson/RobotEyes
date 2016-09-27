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

#include "camera.h"
#include "opticalFlow.h"


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

	//SimpleOpenNIViewer v;
	//v.run();


    // Create the capture object
    // 0 -> input arg that specifies it should take the input from the webcam
    //cv::VideoCapture cap(1);
    //
    //if(!cap.isOpened())
    //{
    //    cerr << "Unable to open the webcam. Exiting!" << endl;
    //    return -1;
    //}
    //
    //// Termination criteria for tracking the points
    //cv::TermCriteria terminationCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.02);

    //// Size of the block that is used for matching
    //cv::Size windowSize(25,25);
    //
    //// Maximum number of points that you want to track
    //const int maxNumPoints = 200;
    //
    //std::string windowName = "Lucas Kanade Tracker";
    //cv::namedWindow(windowName, 1);
    //cv::setMouseCallback(windowName, onMouse, 0);
    //
    //cv::Mat prevGrayImage, curGrayImage, image, frame;
    //std::vector<cv::Point2f> trackingPoints[2];
    //
    //// Image size scaling factor for the input frames
    //float scalingFactor = 0.75;
    //
    //// Iterate until the user hits the Esc key
    //while(true)
    //{
    //    // Capture the current frame
    //    cap >> frame;
    //    
    //    // Check if the frame is empty
    //    if(frame.empty())
    //        break;
    //    
    //    // Resize the frame
    //    resize(frame, frame, cv::Size(), scalingFactor, scalingFactor, cv::INTER_AREA);
    //    
    //    // Copy the input frame
    //    frame.copyTo(image);
    //    
    //    // Convert the image to grayscale
    //    cv::cvtColor(image, curGrayImage, cv::COLOR_BGR2GRAY);
    //    
    //    // Check if there are points to track
    //    if(!trackingPoints[0].empty())
    //    {
    //        // Status vector to indicate whether the flow for the corresponding features has been found
    //        std::vector<uchar> statusVector;
    //        
    //        // Error vector to indicate the error for the corresponding feature
    //        std::vector<float> errorVector;
    //        
    //        // Check if previous image is empty
    //        if(prevGrayImage.empty())
    //        {
    //            curGrayImage.copyTo(prevGrayImage);
    //        }
    //        
    //        // Calculate the optical flow using Lucas-Kanade algorithm
    //        calcOpticalFlowPyrLK(prevGrayImage, curGrayImage, trackingPoints[0], trackingPoints[1], statusVector, errorVector, windowSize, 3, terminationCriteria, 0, 0.001);

    //        int count = 0;
    //        
    //        // Minimum distance between any two tracking points
    //        int minDist = 7;
    //        
    //        for(int i=0; i < trackingPoints[1].size(); i++)
    //        {
    //            if(pointTrackingFlag)
    //            {
    //                // If the new point is within 'minDist' distance from an existing point, it will not be tracked
    //                if(norm(currentPoint - trackingPoints[1][i]) <= minDist)
    //                {
    //                    pointTrackingFlag = false;
    //                    continue;
    //                }
    //            }
    //            
    //            // Check if the status vector is good
    //            if(!statusVector[i])
    //                continue;
    //            
    //            trackingPoints[1][count++] = trackingPoints[1][i];

    //            // Draw a filled circle for each of the tracking points
    //            int radius = 8;
    //            int thickness = 2;
    //            int lineType = 8;
    //            cv::circle(image, trackingPoints[1][i], radius, cv::Scalar(0,255,0), thickness, lineType);
    //        }
    //        
    //        trackingPoints[1].resize(count);
    //    }
    //    
    //    // Refining the location of the feature points
    //    if(pointTrackingFlag && trackingPoints[1].size() < maxNumPoints)
    //    {
    //        std::vector<cv::Point2f> tempPoints;
    //        tempPoints.push_back(currentPoint);
    //        
    //        // Function to refine the location of the corners to subpixel accuracy.
    //        // Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
    //        cornerSubPix(curGrayImage, tempPoints, windowSize, cvSize(-1,-1), terminationCriteria);
    //        
    //        trackingPoints[1].push_back(tempPoints[0]);
    //        pointTrackingFlag = false;
    //    }
    //    
    //    // Display the image with the tracking points
    //    imshow(windowName, image);
    //    
    //    // Check if the user pressed the Esc key
    //    char ch = cv::waitKey(10);
    //    if(ch == 27)
    //        break;
    //    
    //    // Swap the 'points' vectors to update 'previous' to 'current'
    //    std::swap(trackingPoints[1], trackingPoints[0]);
    //    
    //    // Swap the images to update previous image to current image
    //    cv::swap(prevGrayImage, curGrayImage);
    //}
    //
    //return 0;

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