#include "opticalFlow.h"

cv::Mat OpticalFlow::prevGrayImage;
bool pointTrackingFlag = false;

cv::Point2f currentPoint;

// Function to detect mouse events
void 
onMouse(int event, int x, int y, int, void*)
{
	// Detect the mouse button down event
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		// Assign the current (x,y) position to currentPoint
		currentPoint = cv::Point2f((float)x, (float)y);

		// Set the tracking flag
		pointTrackingFlag = true;
	}
}

// Function to calculate the positions of trackingPoints
std::vector<std::vector<cv::Point2f>> 
OpticalFlow::getTrackingPositions(cv::Mat image, std::vector<std::vector<cv::Point2f>> trackingPoints)
{
	// Convert the image to grayscale
	cv::cvtColor(image, curGrayImage, cv::COLOR_BGR2GRAY);

	// Check if there are points to track
	if (!trackingPoints[0].empty())
	{
		// Status vector to indicate whether the flow for the corresponding features has been found(1)
		std::vector<uchar> statusVector;

		// Error vector to indicate the error for the corresponding feature
		std::vector<float> errorVector;

		// Check if previous image is empty
		if (prevGrayImage.empty())
		{
			curGrayImage.copyTo(prevGrayImage);
		}

		// Calculate the optical flow using Lucas-Kanade algorithm
		calcOpticalFlowPyrLK(prevGrayImage, curGrayImage, trackingPoints[0], trackingPoints[1], statusVector, errorVector, windowSize, 3, terminationCriteria, 0, 0.001);

		int count = 0;

		// Minimum distance between any two tracking points
		int minDist = 7;

		for (int i = 0; i < trackingPoints[1].size(); i++)
		{
			if (pointTrackingFlag)
			{
				// If the new point is within 'minDist' distance from an existing point, it will not be tracked
				if (norm(currentPoint - trackingPoints[1][i]) <= minDist)
				{
					pointTrackingFlag = false;
					continue;
				}
			}

			// Check if the status vector is good
			if (!statusVector[i])
				continue;

			trackingPoints[1][count++] = trackingPoints[1][i];

			// Draw a filled circle for each of the tracking points
			int radius = 8;
			int thickness = 2;
			int lineType = 8;
			cv::circle(image, trackingPoints[1][i], radius, cv::Scalar(0, 255, 0), thickness, lineType);
		}

		trackingPoints[1].resize(count);
	}

	// Refining the location of the feature points
	if (pointTrackingFlag && trackingPoints[1].size() < maxNumPoints)
	{
		std::vector<cv::Point2f> tempPoints;
		tempPoints.push_back(currentPoint);

		// Function to refine the location of the corners to subpixel accuracy.
		// Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
		cornerSubPix(curGrayImage, tempPoints, windowSize, cvSize(-1, -1), terminationCriteria);

		trackingPoints[1].push_back(tempPoints[0]);
		pointTrackingFlag = false;
	}


	// Swap the 'points' vectors to update 'previous' to 'current'
	std::swap(trackingPoints[1], trackingPoints[0]);

	// Swap the images to update previous image to current image
	cv::swap(prevGrayImage, curGrayImage);

	return trackingPoints;
}