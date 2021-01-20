
int main(int argc, char** argv) {
	std::string name_files = ".\\YOLO\\coco.names";
	std::string modelConfiguration = ".\\YOLO\\yolov3.cfg";
	std::string modelWeights = ".\\YOLO\\yolov3.weights";
	
	Objectdection image_dection(name_files, modelConfiguration, modelWeights);
	cv::VideoCapture cap;
	std::vector<Rect> boxes;
	std::string image_path = ".\\YOLO\\IMG_1795.JPG";
	std::string type = "image";
	image_dection.image_input(image_path, type, cap);
	cv::Mat frame, img;
	img = imread(image_path);
	cap >> frame;
	while (cv::waitKey(30) != 27) {
		cv::imshow("out", frame);
	}
	image_dection.imageTo4D(frame);
	image_dection.compute_image();
	image_dection.postprocess(boxes);
	for (auto& ele : boxes) {
		cout << "Height: " << ele.height << " Width: " << ele.width << " x: " << ele.x << " y: " << ele.y << endl;
	}
	return 0;
}
