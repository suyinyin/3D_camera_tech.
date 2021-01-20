#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

int main(int argc, char** argv) {
	cv::VideoCapture cap;
	std::string video_path = "E:\\PayloadTestVideo_Jan14_2021\\Crop_Video\\de3in3_P60N75_Trim.mp4";
	cap.open(video_path);
	cv::Mat frame, frame_process;
	// input video size
	int width = 1920, height = 1080;
	// output video size;
	width = width / 2;
	height = height / 2;
	while (cv::waitKey(30) != 27) {
		cap >> frame;

		
		cv::resize(frame, frame_process, cv::Size(width, height), 0, 0, cv::INTER_CUBIC);
		cv::resizeWindow("frame", width, height);
		cv::imshow("frame", frame_process);





	}








	return 0;
}

