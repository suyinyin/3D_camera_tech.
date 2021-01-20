#include"Balltracking.h"

Balltracking::Balltracking(const int& color_flag) {

    switch (color_flag)
    {
    case 0:
        ball_color_hsv_min = Orange_COLOR_HSV_MIN;
        ball_color_hsv_max = Orange_COLOR_HSV_MAX;
        color = Orange_RGB;
        break;
    case 1:
        ball_color_hsv_min = Green_COLOR_HSV_MIN;
        ball_color_hsv_max = Green_COLOR_HSV_MAX;
        color = Green_RGB;
        break;
    case 2:
        ball_color_hsv_min = Blue_COLOR_HSV_MIN;
        ball_color_hsv_max = Blue_COLOR_HSV_MAX;
        color = Blue_RGB;
        break;
    case 3:
        ball_color_hsv_min = Pink_COLOR_HSV_MIN;
        ball_color_hsv_max = Pink_COLOR_HSV_MAX;
        color = Pink_RGB;
        break;
    case 4:
        ball_color_hsv_min = BALL_COLOR_HSV_MIN;
        ball_color_hsv_max = BALL_COLOR_HSV_MAX;
        color = Blue_RGB;
        break;
    default:
        break;
    }
    
}
    

Balltracking::Balltracking(const Balltracking& that) {
    frame = that.frame;
    roi = that.roi;
    temp_size = that.temp_size;
    ball_color_hsv_min = that.ball_color_hsv_min;
    ball_color_hsv_max = that.ball_color_hsv_max;
    color = that.color;
}

Balltracking& Balltracking::operator=(const Balltracking& that) {
    if (this != &that) {
        frame = that.frame;
        roi = that.roi;
        temp_size = that.temp_size;
        ball_color_hsv_min = that.ball_color_hsv_min;
        ball_color_hsv_max = that.ball_color_hsv_max;
        color = that.color;
    }
    return *this;

}


Balltracking::~Balltracking() {


}

// TEMPORARY: we reduce frame size
void Balltracking::convert_frame(cv::Mat& frame_input, const int& width, const int& height) {
    
    // setting processing frame size.
    temp_size.width = width;
    temp_size.height = height;

    //we reduce frame size
    cv::resize(frame_input, frame, temp_size, 0, 0, INTER_CUBIC);
}


/**
    Compute the ROI where to search the ball, given a ball position estimation

    @param position The estimated position of the ball
    @param frame_size The total frame size
    @param roi The output rectangle
*/

void  Balltracking::getRoiRect(Point position, cv::Mat& roi_out) {
    cv::Size frame_size = temp_size;
    int x_min = position.x - (ROI_WIDTH / 2);
    if (x_min < 0)
        x_min = 0;

    int x_max = position.x + (ROI_WIDTH / 2);
    if (x_max > frame_size.width - 1)
        x_max = frame_size.width - 1;

    int y_min = position.y - (ROI_HEIGHT / 2);
    if (y_min < 0)
        y_min = 0;

    int y_max = position.y + (ROI_HEIGHT / 2);
    if (y_max > frame_size.height - 1)
        y_max = frame_size.height - 1;

    roi.x = x_min;
    roi.y = y_min;
    roi.width = x_max - x_min;
    roi.height = y_max - y_min;
    roi_out = cv::Mat(frame, roi);
}

// clear roi reactangle
void  Balltracking::clear_roi() {
    roi.x = 0;
    roi.y = 0;
    roi.width = 0;
    roi.height = 0;
}


/**
    Apply a segmentation with a threshold based on the color of the ball

    @param roi_bgr_blurred The ROI blurred, formatted as BGR
    @param roi_binarized The output binarized frame
*/

void Balltracking::thresholdSegmentation(Mat& roi_bgr_blurred, Mat& roi_binarized) {
    Mat roi_hsv(Size(roi_bgr_blurred.cols, roi_bgr_blurred.rows), CV_8UC3);

    // we convert the frame to HSV
    cvtColor(roi_bgr_blurred, roi_hsv, COLOR_BGR2HSV);

    // we compute the mask by binarizing the picture with a color threshold
    inRange(roi_hsv, ball_color_hsv_min, ball_color_hsv_max, roi_binarized);


    //cv::erode(roi_binarized, roi_binarized, cv::Mat(), cv::Point(-1, -1), 2);
    //cv::dilate(roi_binarized, roi_binarized, cv::Mat(), cv::Point(-1, -1), 2);
    // we apply a closing (dilatation then erosion)
    morphologyEx(roi_binarized, roi_binarized, MORPH_CLOSE,getStructuringElement(MORPH_ELLIPSE, Size(CLOSING_KERNEL_LENGTH, CLOSING_KERNEL_LENGTH)));

}


/**
    Detect the ball position with a Hough transform on a binarized picture

    @param roi_binarized The binarized picture of the ROI
    @param roi_rect The Rect with the coordinates of the ROI (needed to compute center of circles)
    @param circles The output list of circles
*/
void Balltracking::detectBallWithHough(Mat& roi_binarized, vector<Vec3f>& circles) {
    circles.clear();
    // we apply a Canny edge detector
    Canny(roi_binarized, roi_binarized, 50, 200, 3);
    vector<Vec3f> circles_detection;
    // we apply the circle Hough transform. Also, you can use  cv.findContours() to find other contours in the input image
    HoughCircles(roi_binarized, circles_detection, HOUGH_GRADIENT, 1, 10, 200, 15, 0, 0);

    // for each circle, we correct the center position
    // (the HoughCircles function gives us the position inside the ROI,
    //  whereas we want the position inside the full frame)
    for (size_t i = 0; i < circles_detection.size(); i++) {
        Vec3f circle = circles_detection.at(i);
        circle[0] = circle[0] + roi.x;
        circle[1] = circle[1] + roi.y;
        if((circle[0]<= (temp_size.width-1)) && (circle[1] <= (temp_size.height-1)))
            circles.push_back(circle);

    }
    
}

/**
    Detect the ball position with contours on a binarized picture

    @param roi_binarized The binarized picture of the ROI
    @param roi_rect The Rect with the coordinates of the ROI (needed to compute center of circles)
    @param circles The output list of possible ball positions
*/
void Balltracking::detectBallWithContours(Mat& roi_binarized, vector<Point>& positions) {
    positions.clear();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy; // unused

    // first, we search for contours around shapes in the binarized ROI
    findContours(roi_binarized, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // we compute the centroid of each shape
    for (size_t i = 0; i < contours.size(); i++) {
        Moments m = moments(contours.at(i));
        Point centroid;
        centroid.x = (int)(m.m10 / m.m00);
        centroid.y = (int)(m.m01 / m.m00);

        // sometimes the results are impossible values, so we ignore them
        if (centroid.x < 0 || centroid.x >= roi_binarized.cols
            || centroid.y < 0 || centroid.y > roi_binarized.rows)
            continue;

        // we correct the center position (we computed the position in the ROI,
        // we want the position in the full frame)
        centroid.x += roi.x;
        centroid.y += roi.y;

        // if the centroid is out of image range, ignore it.
        if (centroid.x >= (temp_size.width - 1) || centroid.y >= (temp_size.height - 1))
            continue;
        positions.push_back(centroid);
    }
}


// draws the circles that found with Hough transform
void Balltracking::drawHoughCircles(Mat& frame_show, vector<Vec3f> circles) {
    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        circle(frame_show, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }
}

// draws the trajectory and the rectangle of the ROI
void Balltracking::drawTrackingInfo_roi(Mat& frame_show, vector<Point> positions) {
    drawTrackingInfo_trajectory(frame_show, positions);
    rectangle(frame_show, roi, Scalar(0, 255, 0));
}


// draws the trajectory
void Balltracking::drawTrackingInfo_trajectory(Mat& frame_show, vector<Point> positions) {

    // we draw the trajectory lines between the positions detected
    for (int j = 0; j < ((int)positions.size() - 1); j++)
        line(frame_show, positions.at(j), positions.at(j + 1), Scalar(0, 0, 255), 2, LINE_AA);
    // we draw a circle for each position detected
    for (size_t j = 0; j < positions.size(); j++)
        circle(frame_show, positions.at(j), 2, Scalar(255, 0, 0), 2);

}
