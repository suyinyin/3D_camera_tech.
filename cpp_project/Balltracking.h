#pragma once
#ifndef BALLTRACKING_
#define BALLTRACKING_

/*
	--------------------------------------------------------Class description----------------------------------------------------
	*************************************************************************
	**    Ball tracking                                                    **
	**	  Refer to https://github.com/vmarquet/table-tennis-computer-vision/tree/master/ball_tracking		   **
	**    Created on 23rd Dec., 2020				                           **
	**    Author: Yinyin SU												   **
	**    E-mail: yinyinsu1991@gmail.com								   **
	*************************************************************************


*/

// import all other supplementary libraries.
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>


using namespace cv;
using namespace std;
using namespace std::chrono;
// define namespace

#define SHOW_WINDOWS  // if defined, display a window with image result for each step

// we use only orange balls, like RGB = [255, 252, 31]
// For this color, HSV is [30, 224, 255]
// We want to ignore the V of HSV, because we don't want to be dependent
// of the amount of light

 // new environments
// orange
const Scalar Orange_COLOR_HSV_MIN(3, 130, 123);
const Scalar Orange_COLOR_HSV_MAX(20, 255, 255);
const Scalar Orange_RGB(51, 155, 255);

//Green
const Scalar Green_COLOR_HSV_MIN(88, 101, 46);
const Scalar Green_COLOR_HSV_MAX(97, 255, 255);
const Scalar Green_RGB(0, 255, 0);

//Blue
const Scalar Blue_COLOR_HSV_MIN(99, 108, 37);
const Scalar Blue_COLOR_HSV_MAX(112, 255, 255);
const Scalar Blue_RGB(255, 0, 0);

// pink
const Scalar Pink_COLOR_HSV_MIN(56, 82, 40);
const Scalar Pink_COLOR_HSV_MAX(73, 255, 255);
const Scalar Pink_RGB(204, 0, 204);


/*
// orange
const Scalar Orange_COLOR_HSV_MIN(3, 130, 121);
const Scalar Orange_COLOR_HSV_MAX(20, 255, 255);
const Scalar Orange_RGB(51,155,255);

//Green
const Scalar Green_COLOR_HSV_MIN(89, 101, 92);
const Scalar Green_COLOR_HSV_MAX(97, 255, 255);
const Scalar Green_RGB(0, 255, 0);

//Blue
const Scalar Blue_COLOR_HSV_MIN(101, 152, 62);
const Scalar Blue_COLOR_HSV_MAX(128, 255, 255);
const Scalar Blue_RGB(255, 0, 0);

// pink
const Scalar Pink_COLOR_HSV_MIN(40, 83, 50);
const Scalar Pink_COLOR_HSV_MAX(68, 255, 255);
const Scalar Pink_RGB(204, 0, 204);
*/
// test(orange)
const Scalar BALL_COLOR_HSV_MIN(3, 137, 136);
const Scalar BALL_COLOR_HSV_MAX(14, 255, 255);


// the size of the kernels used for blurring / morphological operations
const int BLUR_KERNEL_LENGTH = 13;
const int CLOSING_KERNEL_LENGTH = 7;

// the size of the zone we will use to search for the ball,
// if we know the position of the ball in the previous frame
const int BALL_SIZE = 7;  // pixels
const int ROI_WIDTH = BALL_SIZE * 10;
const int ROI_HEIGHT = BALL_SIZE * 10;


class Balltracking {
public:
	Balltracking(const int& color_flag);
	Balltracking(const Balltracking& that);
	Balltracking& operator=(const Balltracking& that);
	virtual ~Balltracking();
	void convert_frame(cv::Mat& frame_input, const int& width, const int& height);
	void getRoiRect(Point position, cv::Mat& roi_out);
	void clear_roi();
	void thresholdSegmentation(Mat& roi_bgr_blurred, Mat& roi_binarized);
	void detectBallWithHough(Mat& roi_binarized, vector<Vec3f>& circles);
	void drawHoughCircles(Mat& frame, vector<Vec3f> circles);
	void detectBallWithContours(Mat& roi_binarized, vector<Point>& positions);
	void drawTrackingInfo_trajectory(Mat& frame, vector<Point> positions);
	void drawTrackingInfo_roi(Mat& frame, vector<Point> positions);

private:
	cv::Mat frame;
	cv::Size temp_size;
	cv::Rect roi; //get roi
	cv::Scalar color;
	cv::Scalar  ball_color_hsv_min;
	cv::Scalar  ball_color_hsv_max;
};

#endif