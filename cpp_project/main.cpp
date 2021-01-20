
#include"Realsense.h"
#include "Balltracking.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include"3Drendering.h"

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
    /*
    // Initialize window for rendering
    window app(window_width, window_height, "Ball position", window_position[4][0], window_position[4][1]);
    // Construct an object to manage view state
    glfw_state app_state(0.0, 0.0);
    // Register callbacks to allow manipulation of the view state
    register_glfw_callbacks(app, app_state);
    std::vector<rs2_vector> trajectory;
    */
    // Build realsense object with bag file index
    int bag_index = 3;
    Realsense my_realsenses(bag_index);
    // Build Balltracking object to track an arange. 0-orange, 1-green, 2-blue, 3-pink, 4-test(orange)
    int color_index = 0;
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
    string positions_path = ".\\exportData\\ball_position\\color_map\\20210118\\" + std::to_string(bag_index) + "\\" + color[color_index] + "_points_uv_xyz.dat";
    ofstream output_position_uv(positions_path, std::ios::out);
    bool ball_found_prev = false;  // whether we found ball during previous iteration
    cv::Mat frame_show, roi_binarized_to_show;
    int file_index = 0;
    //while (app && cv::waitKey(30) != 27)
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
        // if read bag file from local, this function will work.
        /*
        // Configure scene, draw floor, handle manipultation by the user etc.
        render_scene(app_state);
        draw_axes(0.5,0.8);
        */
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
        /*
        if (circles.size()==1) {
            ball_found = true;
            ball_position = Point(circles[0][0], circles[0][1]);
            orange.drawHoughCircles(frame, circles);
        }
        */

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
            if (possible_positions.size()==1) {
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
        if (ball_found_prev)
            orange.drawTrackingInfo_roi(frame, positions);
        else
            orange.drawTrackingInfo_trajectory(frame, positions);
        // Store color frame after tracking
        frame_to_show.push_back(frame);
        ball_found_prev = ball_found;
        auto time_point8 = std::chrono::steady_clock::now();
       /*
        for (int i = 0; i < frame_to_show.size(); ++i) {
            cv::resizeWindow(window_name[i], window_width, window_height);
            cv::moveWindow(window_name[i], window_position[i][0], window_position[i][1]);
            cv::putText(frame_to_show[i], "Time: " + time_stamp + " ms", cv::Point(2, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(178, 255, 102), 1, 8, 0);
            cv::imshow(window_name[i], frame_to_show[i]);
        }
        */
        
        auto time_point9 = std::chrono::steady_clock::now();
        //std::cout << "Time elapse 5 (videoshow): " << std::chrono::duration_cast<std::chrono::milliseconds>(time_point9 - time_point8).count() << std::endl;

        // store 3d coordinate value of all frame. Units: milimeter 
        std::vector<int> point_3d_value = {0,0,0};
        rs2_vector point_3d_tra;
        // get 3d points from the depth and color images, and do not save PLY file.
        unsigned short depth_value = 0;
        // Not saving uneffective points
        if (ball_position.x == 0 || ball_position.y == 0) continue;
        my_realsenses.get_points_cloud(file_index, depth_value, ball_position.x, ball_position.y, 0);
        
        if (depth_value == 0) continue;
        //my_realsenses.save_points_cloud(file_index);
        my_realsenses.get_point_3Dvalue(point_3d_value, ball_position.x, ball_position.y);
        /*
        // render the ball trajectory
        point_3d_tra.x = (float)point_3d_value[0] / 100; 
        point_3d_tra.y = (float)point_3d_value[1] / 100; 
        point_3d_tra.z = (float)point_3d_value[2] / 100 - 5;
        trajectory.push_back(point_3d_tra);
        draw_trajectory(trajectory);
        */
        // output the uv value of all points
        if (ball_position.x != 0 && ball_position.y != 0) {
            output_position_uv << time_stamp << "\t" << setw(5) << ball_position.x << setw(5) << ball_position.y <<
                setw(7) << point_3d_value[0] << setw(7) << point_3d_value[1] << setw(7) << point_3d_value[2] << std::endl;
        }
        
        auto time_point10 = std::chrono::steady_clock::now();
        //std::cout << "Time elapse 6 (save to disk): " << std::chrono::duration_cast<std::chrono::milliseconds>(time_point10 - time_point9).count() << std::endl;
        file_index++;
    }

    //saving points uv value completely.
    output_position_uv.close();

    return 0;
}

