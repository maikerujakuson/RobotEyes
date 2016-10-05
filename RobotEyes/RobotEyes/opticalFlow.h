#ifndef __OPTICALFLOW_H_INCLUDED__
#define __OPTICALFLOW_H_INCLUDED__

#include<opencv2\opencv.hpp>

extern bool pointTrackingFlag;
extern cv::Point2f currentPoint;
extern void onMouse(int event, int x, int y, int, void*);

class OpticalFlow {
public:
	OpticalFlow(std::string windowName) : windowSize(25, 25), scalingFactor(0.75), terminationCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.02)
	{
		cv::setMouseCallback(windowName, onMouse, 0);
	};
	cv::Point2f currentPoint;
	bool pointTrackingFlag = false;
	std::vector<std::vector<cv::Point2f>> getTrackingPositions(cv::Mat image, std::vector<std::vector<cv::Point2f>> trackingPOints);
	/*void onMouse(int event, int x, int y, int, void*);*/

private:
	static cv::Mat prevGrayImage;
	cv::Mat curGrayImage, frame;
	cv::Size windowSize;
	// Maximum number of points that you want to track
	const int maxNumPoints = 200;
	// Image size scaling factor for the input frames
	float scalingFactor;
	// Termination criteria for tracking the points
	cv::TermCriteria terminationCriteria;
};

#endif