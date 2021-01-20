
#include"Realsense.h"
#include "Balltracking.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include"3Drendering.h"
#include <opencv2/videoio/videoio_c.h>

using namespace cv;
using namespace std;

int main(int argc, char const* argv[])
{
    // set image show windows size and position
    int window_width = 640, window_height = 480;
    std::vector <std::string> window_name{ "Color live", "Aligned depth live", "threshold segmentation live", "Ball tracking","3D points position" };
    std::vector<std::vector<int>> window_position{ {0, 0},{ window_width * 1, 0},{ window_width * 2, 0}, {0, window_height * 1 + 30}, { window_width * 1, window_height * 1 + 30} };
    // obtain time stamp for each frame
    std::string time_stamp;
    // Build realsense object with bag file index
    int bag_index = 3;
    Realsense my_realsenses(bag_index);
    // Build Balltracking object to track an arange. 0-orange, 1-green, 2-blue, 3-pink, 4-test(orange)
    int color_index = 2;
    std::vector<std::string> color = { "orange", "green", "blue", "pink" };
    Balltracking orange(color_index);
    // Temporarily store all frame to show.
    std::vector<cv::Mat> frame_to_show;
    Mat frame, align_frame, roi_blurred, roi_binarized;
    vector<Vec3f> circles;
    // set objective width and heigth for input image
    const int to_width = 640, to_height = 480;
    vector<Point> positions;  // the history of all detected positions (0 or 1 per frame) 
    // save all positions to file
    bool ball_found_prev = false;  // whether we found ball during previous iteration
    cv::Mat frame_show, roi_binarized_to_show;
    //define object roi
    std::vector<cv::Rect> object_rois;
    std::vector<Point> object_center = { Point(296, 300), Point(369, 238), Point(412, 150),Point(384, 142), Point(338,164), Point(217, 254), Point(160,201)};
    //std::vector<Point> object_center = { Point(296, 301), Point(368, 241), Point(413, 153),Point(384, 145), Point(340,164), Point(218, 256), Point(160,200)};
    int object_size = 15;
    for (int object_index = 0; object_index < object_center.size(); object_index++) {
        cv::Rect object_roi;
        object_roi.x = object_center[object_index].x - object_size;
        object_roi.y = object_center[object_index].y - object_size;
        object_roi.width = object_size*2;
        object_roi.height = object_size * 2;
        object_rois.push_back(object_roi);
    }

    int file_index = 0;
    cv::VideoWriter video("out_repeat_3_origin.mp4", CV_FOURCC('m', 'p', '4', 'v'), 30, Size(to_width, to_height));
    while (cv::waitKey(30) != 27)
    {
        // clear frame storage 
        frame_to_show.clear();
        bool ball_found = false;
        Point ball_position;

        auto time_point1 = std::chrono::steady_clock::now();
        
        //get frame from realsense
        if (!my_realsenses.retrieve_frame()) break;
        // get color image from frame
        my_realsenses.get_color_image();


        //align the depth image to color frame
        my_realsenses.align_image();
        // obtain color frame and aligned frame from realsense
        my_realsenses.out_depth_color_to_main(frame, align_frame, time_stamp);
        auto time_point2 = std::chrono::steady_clock::now();
        //std::cout << "Time elapse 1: " << std::chrono::duration_cast<std::chrono::milliseconds>(time_point2 - time_point1).count() << std::endl;
        // Store the color and aligned frame to show
        frame_show = frame.clone();
        //cv::putText(frame_show, "Time: " + time_stamp + " ms", cv::Point(2,10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(178, 255, 102), 1, 8, 0);
        frame_to_show.push_back(frame_show);
       
        frame_to_show.push_back(align_frame);

        orange.convert_frame(frame, to_width, to_height);

        auto time_point3 = std::chrono::steady_clock::now();
        // if we found the ball during previous iteration, we use these coordinates
        // as centre of a reduced ROI, else the ROI is the full frame to inprove efficiency
        cv::Mat roi;
        if (ball_found_prev)
            orange.getRoiRect(positions.back(), roi);
        else
        {
            roi = frame;
            orange.clear_roi();
        }
        // we blur the picture to remove the noise
        GaussianBlur(roi, roi_blurred, Size(BLUR_KERNEL_LENGTH, BLUR_KERNEL_LENGTH), 0, 0);
        auto time_point4 = std::chrono::steady_clock::now();
        //std::cout << "Time elapse 2 (GaussianBlur): " << std::chrono::duration_cast<std::chrono::milliseconds>(time_point4 - time_point3).count() << std::endl;

        auto time_point5 = std::chrono::steady_clock::now();
        // ===== first try =====
        // with a threshold segmentation and a Hough transform
        orange.thresholdSegmentation(roi_blurred, roi_binarized);
        roi_binarized_to_show = roi_binarized.clone();
        // Store the segmentation frame to show
        frame_to_show.push_back(roi_binarized_to_show);
        orange.detectBallWithHough(roi_binarized, circles);
        // if the number of circles found by the Hough transform is exactly 1,
        // we accept that circle as the correct ball position if (circles.size() == 1)
        if (circles.size() <= 3 && circles.size() != 0) {
            // find the max radius circle and used as the dectected ball
            float radius = 0;
            int max_index = 0;
            for (int index_circles = 0; index_circles < circles.size(); ++index_circles) {
                if (circles[index_circles][2] > radius) {
                    radius = circles[index_circles][2];
                    max_index = index_circles;
                }
            }

            ball_found = true;
            ball_position = Point(circles[max_index][0], circles[max_index][1]);
            std::vector<cv::Vec3f> detect_circle = { circles[max_index] };
            orange.drawHoughCircles(frame, detect_circle);
        }
        std::cout << "circles: " << circles.size() << endl;

        auto time_point6 = std::chrono::steady_clock::now();

        //std::cout << "Time elapse 3 (thresholdSegmentation): " << std::chrono::duration_cast<std::chrono::milliseconds>(time_point6 - time_point5).count() << std::endl;

        if (ball_found == false) {
            // ===== second try =====
            // we use OpenCV convex hull algorithm to detect shapes
            // (the ball is not detected by Hough transform if it is not round enough)
            vector<Point> possible_positions;
            orange.detectBallWithContours(roi_binarized, possible_positions);

            std::cout << "possible_positions: " << possible_positions.size() << endl;

            // if the number of positions found by the Hough transform is exactly 1,
            // we accept that position as the correct ball position
            if (possible_positions.size() == 1) {
                ball_found = true;
                ball_position = possible_positions[0];
            }
        }

        auto time_point7 = std::chrono::steady_clock::now();

        //std::cout << "Time elapse 4 (second try): " << std::chrono::duration_cast<std::chrono::milliseconds>(time_point7 - time_point6).count() << std::endl;
        // if we have found the ball position, we save it in the history
        if (ball_found == true)
            positions.push_back(ball_position);

        // we display the image
       /*  if (ball_found_prev)
            orange.drawTrackingInfo_roi(frame, positions);*/
        // Store color frame after tracking
        frame_to_show.push_back(frame);
        ball_found_prev = ball_found;
        
        int i = 3;
        cv::resizeWindow(window_name[i], window_width, window_height);
        cv::putText(frame_to_show[i], "Time: " + time_stamp + " ms", cv::Point(2, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(178, 255, 102), 1, 8, 0);
        /*
        for (int index_object = 0; index_object < object_rois.size(); ++index_object) {
            rectangle(frame_to_show[i], object_rois[index_object], Scalar(0, 255, 0),2);
            int x = object_center[index_object].x - 7;
            int y = object_center[index_object].y + 7;
            cv::putText(frame_to_show[i],std::to_string(index_object), cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0), 2, 8, 0);
        }
        */
        video.write(frame_to_show[i]);
        cv::imshow(window_name[i], frame_to_show[i]);
        
        file_index++;
    }
    return 0;
}

