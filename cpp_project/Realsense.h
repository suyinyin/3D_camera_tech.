#ifndef REALSENSE_
#define REALSENSE_

/*
	--------------------------------------------------------Class description----------------------------------------------------
	*************************************************************************
	**    Products description for Realsense                               **
	**	  Refer to https://github.com/IntelRealSense/librealsense		   **
	**    Created on 3rd Dec., 2020				                           **
	**    Author: Yinyin SU												   **
	**    E-mail: yinyinsu1991@gmail.com								   **
	*************************************************************************
	Realsense();													/Constructor and configure some information or initilizing the member variables.
	Realsense(const Realsense& that);								/Copy constructor.
	Realsense& operator=(const Realsense& that);					/Assignment constructor.
	virtual ~Realsense();											/Deconstructor.
	void get_a_sensor_from_a_device(const rs2::device& dev);		/Get sensor infos for each device.
	void retrieve_frame();											/Retrieve frame from device.
	bool get_color_image();											/Get color image from the current frame.
	void save_color_image(const int& image_index);					/Save color image as png format by OPENCV lib.
	bool get_depth_image(bool range_index);							/get depth image from the current frame.
	void save_depth_image(const int& image_index);					/Save depth image as png format by OPENCV lib.
	bool get_infrared_image();										/Get infrared image, including left and right stero cameras, from the current frame.
	void save_infrared_image(const int& image_index);				/Save all infrared images as png format by OPENCV lib.
	void crop_image(cv::Mat& showFrame);							/Crop all image by given geometric parameters before saving.
	void save_raw_data(const int& index);							/Save all raw depth infos, left and right infrared infos as dat format for reference. 
	
*/

// import all other supplementary libraries.
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
//#include "3Drendering.h"          // Include short list of convenience functions for rendering
#include <algorithm>            // std::min, std::max

// define namespace
namespace REAL {

	enum STREAMPARAS {
		INITIAL_WIDTH = 640,
		INITIAL_HEIGHT = 480,
		FPS = 30
	};

	enum ROI {
		IMAGE_START_X = 200,
		IMAGE_START_Y = 130,
		IMAGE_WIDTH = 220,
		IMAGE_HEIGHT = 240,
		FLAG_CROP = false

	};

	enum DEPTH {

		IMAGE_DEPTH_MIN = 400,
		IMAGE_DEPTH_MAX = 6000,
		IMAGE_INFRARED_THRESHOLD = 3000,
		DIRECTION = 0,                             // direction = 1, color->depth, direction = 0, depth->color
		BAGFILE = 0,							   // if BAGFILE is 1, the device will record all stream as bag
		FILEFROMOUT = 0			                   // Read bag file from local space
	};

};

using namespace REAL;

class Realsense {
public:
	Realsense(const int& bagfile_index);
	Realsense(const Realsense& that);
	Realsense& operator=(const Realsense& that);
	virtual ~Realsense();
	void get_a_sensor_from_a_device(const rs2::device& dev);
	bool retrieve_frame();
	bool get_color_image();
	void save_color_image(const int& image_index);
	bool get_depth_image(bool range_index = false);
	void save_depth_image(const int& image_index);
	bool get_depth_render_image();
	bool get_infrared_image();
	void save_infrared_image(const int& image_index);
	void crop_image(cv::Mat& showFrame);
	void save_raw_data(const int& index);
	void get_points_cloud(const int& image_index, unsigned short& depth_value, const int& u, const int& v, bool ply_flag); //If ply_flag is 0, ply will not be saved.
	void save_points_cloud(const int& image_index);
	void get_point_3Dvalue(std::vector<int>& point_coordinate, const int& u, const int& v);
	void videoShow();
	void align_image(); 
	void get_save_intrinsics();
	void gettime(int64 timestamp, std::string& Time);
	void out_depth_color_to_main(cv::Mat& color_frame, cv::Mat& align_frame, std::string& time_stamp);
	void chess_connor_point_dection(cv::Mat& color_frame, std::vector<cv::Point2f>& pointbuf, const cv::Size& boardSize);
	//void image_recog();
private:
	rs2::context ctx;
	std::vector<rs2::pipeline>  pipelines;
	std::vector<std::string> cams_serial_NO;
	std::vector<rs2::frameset> frames;
	std::vector<rs2::frameset> aligned_depth_frame;
	std::vector<std::string> time_frame;
	std::vector<cv::Mat> color_images;
	std::vector<cv::Mat> depth_images;
	std::vector<cv::Mat> depth_render_images;
	std::vector<cv::Mat> ir_left_images;
	std::vector<cv::Mat> ir_right_images;
	std::vector<cv::Mat> after_align_images;
	rs2::colorizer cmap;  // Helper to colorize depth images
	int cameras_count = 0;
	rs2::align alignment;
	double alpha = 0.8;
	std::vector<rs2::pipeline_profile> select_intrins;
	rs2::points points; // We want the points object to be persistent so we can display the last cloud when a frame drops
};


#endif // !REALSENSE_

