#pragma once
#ifndef OBJECTDETECTION_
#define OBJECTDETECTION_

/*
	--------------------------------------------------------Class description----------------------------------------------------
	*************************************************************************
	**    Products description for Realsense                               **
	**	  Refer to https://github.com/IntelRealSense/librealsense		   **
	**    Created on 3rd Dec., 2020				                           **
	**    Author: Yinyin SU												   **
	**    E-mail: yinyinsu1991@gmail.com								   **
	*************************************************************************


*/

// import all other supplementary libraries.
#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <chrono> 
using namespace cv;
using namespace dnn;
using namespace std;
using namespace std::chrono;
// define namespace


class Objectdection {
public:
	Objectdection(string& name_files, string& modelConfiguration, string& modelWeights);
	Objectdection(const Objectdection& that);
	Objectdection& operator=(const Objectdection& that);
	virtual ~Objectdection();
	void image_input(const std::string& image_path, const std::string& type, cv::VideoCapture& cap);
	void imageTo4D(cv::Mat& frame);
	vector<String> getOutputsNames();
	void postprocess(vector<Rect>& boxes);
	void compute_image();
	void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);

private:
	double confThreshold = 0.5; // Confidence threshold
	double nmsThreshold = 0.4;  // Non-maximum suppression threshold
	int inpWidth = 416;  // Width of network's input image
	int inpHeight = 416; // Height of network's input image
	vector<string> classes;
	std::string classesFile;
	cv::String modelConfiguration;
	cv::String modelWeights;
	Net net;
	std::string outputFile;
	cv::Mat blob;
	cv::Mat frame;
	vector<Mat> outs;
};


#endif 