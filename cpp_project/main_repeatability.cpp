
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
    std::vector <std::string> window_name{ "Aligned depth live", "orange", "green","blue","pink" };
    std::vector<std::vector<int>> window_position{ {0, 0},{ window_width * 1, 0},{ window_width * 2, 0}, {0, window_height * 1 + 30}, { window_width * 1, window_height * 1 + 30} };
    // obtain time stamp for each frame
    std::string time_stamp;

    // Build realsense object with bag file index
    int bag_index = 15;
    Realsense my_realsenses(bag_index);
    // Build Balltracking object to track an arange. 0-orange, 1-green, 2-blue, 3-pink, 4-test(orange)
    //std::vector<std::string> color = { "orange", "green", "blue", "pink" };
    std::vector<std::string> color = { "orange", "green", "blue", "pink" };
    Balltracking orange(0), green(1), blue(2), pink(3);
    //std::vector<Balltracking> ball_color = { orange, green, blue, pink };
    std::vector<Balltracking> ball_color = { blue, pink };
    // Temporarily store all frame to show.
    std::vector<cv::Mat> frame_to_show;
    Mat frame, align_frame;
    vector<Vec3f> circles;
    vector<Point> possible_positions;
    // set objective width and heigth for input image
    const int to_width = 640, to_height = 480;
    vector<vector<Point>> positions{ 4 };  // the history of all detected positions (0 or 1 per frame)
    // save all positions to file
    string positions_path = ".\\exportData\\ball_position\\color_map\\points_uv_xyz_" + std::to_string(bag_index) + ".dat";
    ofstream output_position_uv(positions_path, std::ios::out);
    std::vector<bool> ball_found_prev = { false, false, false, false }; // whether we found ball during previous iteration
    cv::Mat frame_show, roi_binarized_to_show;
    //VideoWriter video("out_repeat.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(to_width, to_height));
    VideoWriter video("out_repeat.mp4", CV_FOURCC('m', 'p', '4', 'v'), 30, Size(to_width, to_height));
    
    int file_index = 0;
    while (cv::waitKey(30) != 27)
    {
        // clear frame storage 
        frame_to_show.clear();
        std::vector<bool> ball_found = { false, false, false, false };
        //std::vector<Point> ball_position
        std::vector<Point> ball_position = { Point(0,0),Point(0,0),Point(0,0),Point(0,0) };
        
        //get frame from realsense
        my_realsenses.retrieve_frame();
        // get color image from frame
        my_realsenses.get_color_image();
        //align the depth image to color frame
        my_realsenses.align_image();
        // obtain color frame and aligned frame from realsense
        my_realsenses.out_depth_color_to_main(frame, align_frame, time_stamp);
        auto time_point1 = std::chrono::steady_clock::now();
        std::vector<Mat> frames = { frame.clone(),frame.clone(),frame.clone(),frame.clone() };
        auto time_point2 = std::chrono::steady_clock::now();
        std::cout << "Time elapse 1: " << std::chrono::duration_cast<std::chrono::milliseconds>(time_point2 - time_point1).count() << std::endl;
        frame_to_show.push_back(align_frame);
        int index = 0;
        for (auto& ele : ball_color) {
            ele.convert_frame(frames[index], to_width, to_height);
            // if we found the ball during previous iteration, we use these coordinates
            // as centre of a reduced ROI, else the ROI is the full frame to inprove efficiency
            cv::Mat roi, roi_blurred, roi_binarized;
            if (ball_found_prev[index])
                ele.getRoiRect(positions[index].back(), roi);
            else
            {
                roi = frames[index];
                ele.clear_roi();
            }

            // we blur the picture to remove the noise
            GaussianBlur(roi, roi_blurred, Size(BLUR_KERNEL_LENGTH, BLUR_KERNEL_LENGTH), 0, 0);

            // ===== first try =====
        // with a threshold segmentation and a Hough transform
            ele.thresholdSegmentation(roi_blurred, roi_binarized);
            ele.detectBallWithHough(roi_binarized, circles);
            std::cout << "circles: " << circles.size() << endl;

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
                ball_found[index] = true;
                ball_position[index] = Point(circles[max_index][0], circles[max_index][1]);
                std::vector<cv::Vec3f> detect_circle = { circles[max_index] };
                ele.drawHoughCircles(frames[0], detect_circle);
            }

            if (ball_found[index] == false) {
                // ===== second try =====
                // we use OpenCV convex hull algorithm to detect shapes
                // (the ball is not detected by Hough transform if it is not round enough)

                ele.detectBallWithContours(roi_binarized, possible_positions);

                std::cout << "possible_positions: " << possible_positions.size() << endl;

                // if the number of positions found by the Hough transform is exactly 1,
                // we accept that position as the correct ball position
                if (possible_positions.size() == 1 && possible_positions[0].x) {
                    ball_found[index] = true;
                    ball_position[index] = possible_positions[0];
                }
            }

            // if we have found the ball position, we save it in the history
            if (ball_found[index] == true)
                positions[index].push_back(ball_position[index]);

            // we display the image
            if (ball_found_prev[index])
                ele.drawTrackingInfo_roi(frames[0], positions[index]);
            else
                ele.drawTrackingInfo_trajectory(frames[0], positions[index]);

            ball_found_prev[index] = ball_found[index];
            
            
            index++;
        }
        frame_to_show.push_back(frames[0]);
        video.write(frames[0]);
        cv::resizeWindow(window_name[1], window_width, window_height);
        cv::putText(frame_to_show[1], "Time: " + time_stamp + " ms", cv::Point(2, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(178, 255, 102), 1, 8, 0);
        cv::imshow(window_name[1], frame_to_show[1]);



        file_index++;
    }
    //saving points uv value completely.


    return 0;
}

