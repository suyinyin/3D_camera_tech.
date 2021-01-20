#include"Realsense.h"


// constructor
Realsense::Realsense(const int& bagfile_index):alignment(RS2_STREAM_DEPTH){
    pipelines.clear();
    rs2::align alignment_color(RS2_STREAM_COLOR);
    if (!DIRECTION) alignment = alignment_color;
    if (FILEFROMOUT) {
        rs2::pipeline pipe;
        rs2::config cfgs; 
        std::string bag_file_path = "D:\\bag\\realsense_20210118_" + std::to_string(bagfile_index) + ".bag";
        //std::string bag_file_path = ".\\exportData\\bag\\realsense_"+ std::to_string(bagfile_index) + ".bag";
        //std::string bag_file_path = "F:\\experiments_20210109\\bag\\realsense_" + std::to_string(bagfile_index) + ".bag";
        cfgs.enable_device_from_file(bag_file_path, false);

        // set parameters for stream. For the infrared cameras, 1-left stereo camera, 2-right stero camera
        cfgs.enable_stream(RS2_STREAM_COLOR, INITIAL_WIDTH, INITIAL_HEIGHT, RS2_FORMAT_BGR8, FPS);
        cfgs.enable_stream(RS2_STREAM_DEPTH, INITIAL_WIDTH, INITIAL_HEIGHT, RS2_FORMAT_Z16, FPS);
        cfgs.enable_stream(RS2_STREAM_INFRARED, 1, INITIAL_WIDTH, INITIAL_HEIGHT, RS2_FORMAT_Y8, FPS);
        cfgs.enable_stream(RS2_STREAM_INFRARED, 2, INITIAL_WIDTH, INITIAL_HEIGHT, RS2_FORMAT_Y8, FPS);
        cfgs.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        cfgs.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        rs2::pipeline_profile selection = pipe.start(cfgs);
        select_intrins.push_back(selection);
        pipelines.emplace_back(pipe);
    }
    else {
        // Start a streaming pipe per each connected device
        for (auto&& dev : ctx.query_devices())
        {
            rs2::pipeline pipe(ctx);
            rs2::config cfgs;
            std::string device_information = dev.get_info(RS2_CAMERA_INFO_NAME);
            std::cout << "Cameras serial NO." << cameras_count << " " << device_information << std::endl;
            cams_serial_NO.push_back(device_information);
            // Cameras configurations
            cfgs.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            // 

            // set parameters for different sensors(0-Stere Module, 1-RGB Camera, 2-Motion Module)
            //dev.query_sensors()[1].get_option(RS2_OPTION_EXPOSURE);
            // set parameters for stream. For the infrared cameras, 1-left stereo camera, 2-right stero camera
            cfgs.enable_stream(RS2_STREAM_COLOR, INITIAL_WIDTH, INITIAL_HEIGHT, RS2_FORMAT_BGR8, FPS);
            cfgs.enable_stream(RS2_STREAM_DEPTH, INITIAL_WIDTH, INITIAL_HEIGHT, RS2_FORMAT_Z16, FPS);
            cfgs.enable_stream(RS2_STREAM_INFRARED, 1, INITIAL_WIDTH, INITIAL_HEIGHT, RS2_FORMAT_Y8, FPS);
            cfgs.enable_stream(RS2_STREAM_INFRARED, 2, INITIAL_WIDTH, INITIAL_HEIGHT, RS2_FORMAT_Y8, FPS);
            cfgs.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            cfgs.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
            // save all stream as bag file
            if (BAGFILE)
            {
                std::string bag_file_path = "D:\\bag\\realsense_20210118_" + std::to_string(bagfile_index) + ".bag";
                //std::string bag_file_path = "D:\\bag\\realsense_" + std::to_string(bagfile_index) + ".bag";
                cfgs.enable_record_to_file(bag_file_path);

            }

            rs2::pipeline_profile selection = pipe.start(cfgs);
            select_intrins.push_back(selection);
            pipelines.emplace_back(pipe);
            cameras_count++;
        }
    }


}

// copy constructor
Realsense::Realsense(const Realsense& that):alignment(RS2_STREAM_DEPTH) {
    ctx = that.ctx;
    pipelines = that.pipelines;
    cams_serial_NO = that.cams_serial_NO;
    frames = that.frames;
    color_images = that.color_images;
    depth_images = that.depth_images;
    ir_left_images = that.ir_left_images;
    ir_right_images = that.ir_right_images;
    cameras_count = that.cameras_count;
}

// assignment constructor
Realsense& Realsense::operator=(const Realsense& that) {
	if (this != &that) {
        ctx = that.ctx;
        pipelines = that.pipelines;
        cams_serial_NO = that.cams_serial_NO;
        frames = that.frames;
        color_images = that.color_images;
        depth_images = that.depth_images;
        ir_left_images = that.ir_left_images;
        ir_right_images = that.ir_right_images;
        cameras_count = that.cameras_count;
	}
	return *this;
}

// destructor 
Realsense::~Realsense() {
    // Release frames memory
    frames.clear();
    // Release all cameras
    for (int index = 0; index < cams_serial_NO.size();++index) {
        std::cout << "Release the " << index << "-th " << cams_serial_NO[index] << std::endl;
    }
    // Release all devices
    for (auto& ele : pipelines)
        ele.stop();
    pipelines.clear();
}



// get sensor infos from each devices
void Realsense::get_a_sensor_from_a_device(const rs2::device& dev) {
    std::vector<rs2::sensor> sensors = dev.query_sensors();
    std::cout << "Device consists of " << sensors.size() << " sensors:\n" << std::endl;
    int index = 0;
    for (rs2::sensor& sensor : sensors)
    {
        std::string sensor_information;
        if (sensor.supports(RS2_CAMERA_INFO_NAME))
            sensor_information = sensor.get_info(RS2_CAMERA_INFO_NAME);
        std::cout << "  " << index++ << " : " << sensor_information << std::endl;
    }
}

// get and save intrinsic parameters
void Realsense::get_save_intrinsics() {
    std::string save_path = "intrinsics_params.txt";
    std::ofstream intrins(save_path, std::ios::out);
    std::string intrinsics[3] = { "fx  0   x_off", "0   fy  y_off", "0   0   1"};
    std::string distortion[2] = {"Brown-Conrady: [k1, k2, p1, p2, k3]", "F-Theta Fish-eye: [k1, k2, k3, k4, 0]"};
    intrins << "The intrinsic parameters for all devices: " << std::endl;
    intrins << "The internal intrinsic parameters: " << std::endl;
    for (auto& ele:intrinsics)
        intrins << ele << std::endl;
    intrins << "The internal distortion parameters: " << std::endl;
    for (auto& ele : distortion)
        intrins << ele << ", ";
    intrins << "\n\n";
    int device_index = 1;
    for (auto& selection : select_intrins) {
        float internel_paras[3][3] = { 0.0 };
        intrins << "The " << device_index <<"-th device: \n";
        intrins << "Depth: " << std::endl;
        auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        //auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
        auto i = depth_stream.get_intrinsics();
        internel_paras[0][0] = i.fx; internel_paras[0][2] = i.ppx; internel_paras[1][1] = i.fy; internel_paras[1][2] = i.ppy; internel_paras[2][2] = 1.0;
        intrins << "Internal paras: \n";
        for (int row_i = 0; row_i < 3; row_i++) {
            for (int row_j = 0; row_j < 3; row_j++) {
                intrins << std::setw(10) <<internel_paras[row_i][row_j];
            }
            intrins << "\n" ;
        }
        rs2_distortion model = i.model;
        intrins << "Distortion model: " << model <<" paras: ";
        for (int row_i = 0; row_i < 5; row_i++)
            intrins << std::setw(10) << i.coeffs[row_i] << " ";
        intrins << "\n \n";
        
        intrins << "Color: " << std::endl;
        auto color_stream = selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        //auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
        auto i_color = color_stream.get_intrinsics();
        internel_paras[0][0] = i_color.fx; internel_paras[0][2] = i_color.ppx; internel_paras[1][1] = i_color.fy; internel_paras[1][2] = i_color.ppy;
        intrins << "Internal paras: \n";
        for (int row_i = 0; row_i < 3; row_i++) {
            for (int row_j = 0; row_j < 3; row_j++) {
                intrins << std::setw(10) << internel_paras[row_i][row_j];
            }
            intrins << "\n";
        }
        rs2_distortion model_color = i_color.model;
        intrins << "Distortion model: " << model_color << " paras: ";
        for (int row_i = 0; row_i < 5; row_i++)
            intrins << std::setw(10) << i_color.coeffs[row_i] << " ";
        intrins << "\n";

        device_index++;
    }


    intrins.close();
}

// Block program until frames arrive
bool Realsense::retrieve_frame() {
    bool frame_index = 1;
    frames.clear();
    time_frame.clear();
    rs2::frameset frames_tempt;
    std::string Time;
    if (FILEFROMOUT) {
        auto device = select_intrins[0].get_device();
        rs2::playback playback = device.as<rs2::playback>();
        playback.set_real_time(true);
        if (playback.current_status() == 3) frame_index = 0;
    }
    for (auto& pipe : pipelines) {
        //frames_tempt = pipe.wait_for_frames();
        pipe.try_wait_for_frames(&frames_tempt);
        int64 timestamp = (int64)frames_tempt.get_timestamp();
        frames.push_back(frames_tempt);
        gettime(timestamp, Time);
        time_frame.push_back(Time);
    }
    return frame_index;
    
}

// get RGB image
bool Realsense::get_color_image() {
    bool flag = 1;
    color_images.clear();
    for (auto& frame : frames) {
        rs2::video_frame color_image = frame.get_color_frame();
        const void* buffer = color_image.get_data();
        if (buffer) {
            int image_width = color_image.get_width();
            int image_height = color_image.get_height();
            int image_bits = color_image.get_bits_per_pixel();
            cv::Mat color_image_mat(cv::Size(image_width, image_height), CV_8UC3, (void*)buffer, cv::Mat::AUTO_STEP);
            color_images.push_back(color_image_mat);
        }
        else {
            flag = 0;
            std::cout << "Failed to get color image from capture" << std::endl;
            break;
        }
    }
    return flag;
}


// save RGB images
void Realsense::save_color_image(const int& image_index) {
    int camera_index = 0;
    for (auto& color_mat : color_images) {
        //crop_image(color_mat);
        std::cout << "save color image at: " << image_index << std::endl;
        std::string image_color_name = ".\\exportData\\color\\ca" + std::to_string(camera_index) + "p" + std::to_string(image_index) + ".png";
        cv::imwrite(image_color_name, color_mat);
        camera_index++;
    }
    color_images.clear();
}

// get Raw Depth image
bool Realsense::get_depth_image(bool range_index) {
    
    bool flag = 1;
    depth_images.clear();
    for (auto& frame : frames) {
        rs2::depth_frame depth_image = frame.get_depth_frame();
        const void* buffer = depth_image.get_data();
        if (buffer) {
            unsigned int image_width = depth_image.get_width();
            unsigned int image_height = depth_image.get_height();
            unsigned int image_bits = depth_image.get_bits_per_pixel();
            if (range_index) {
                cv::Mat depth_image_mat(cv::Size(image_width, image_height), CV_16UC1, cv::Scalar::all(0));
                for (unsigned int i = 0; i < image_height; ++i) {
                    for (unsigned int j = 0; j < image_width; ++j) {
                        unsigned short depth_value = *((unsigned short*)buffer+ i * image_width + j);
                       if (depth_value > IMAGE_DEPTH_MIN && depth_value < IMAGE_DEPTH_MAX)
                            depth_image_mat.at<unsigned short>(i, j) = (unsigned short)depth_value;
                    }
                }
                depth_images.push_back(depth_image_mat);
            }
            else {
                cv::Mat depth_image_mat(cv::Size(image_width, image_height), CV_16UC1, (void*)buffer, cv::Mat::AUTO_STEP);
                depth_images.push_back(depth_image_mat);
            }
        }
        else {
            flag = 0;
            std::cout << "Failed to get depth image from capture" << std::endl;
            break;
        }
    }
    return flag;
}

// save Depth images(PNG format will leads to missing depth information)
void Realsense::save_depth_image(const int& image_index) {
    int camera_index = 0;
    for (auto& depth_mat : depth_images) {
        //crop_image(color_mat);
        std::cout << "save depth image at: " << image_index << std::endl;
        std::string image_depth_name = ".\\exportData\\depth\\ca" +std::to_string(camera_index)+ "_" + "p" +std::to_string(image_index) + ".png";
        cv::imwrite(image_depth_name, depth_mat);
        camera_index++;
    }
    depth_images.clear();
}

// Get depth image with color render
bool Realsense::get_depth_render_image() {                   
    bool flag = 1;
    depth_render_images.clear();
    for (auto& frame : frames) {
        //rs2::frame  depth_image = frame.get_depth_frame().apply_filter(cmap);
        rs2::depth_frame depth_image = frame.get_depth_frame();
        rs2::video_frame  colorized_depth = cmap.colorize(depth_image);
        const void* buffer = colorized_depth.get_data();
        if (buffer) {
            unsigned int image_width = depth_image.as<rs2::video_frame>().get_width();
            unsigned int image_height = depth_image.as<rs2::video_frame>().get_height();
            cv::Mat depth_render_mat(cv::Size(image_width, image_height), CV_8UC3, (void*)buffer, cv::Mat::AUTO_STEP);
            depth_render_images.push_back(depth_render_mat);
        }
        else {
            flag = 0;
            break;
        }
    }
    return flag;
}

// get infrared image from two stereo cameras.
bool get_infrared(const rs2::frameset& frame, const int& stereo_camera, cv::Mat& ir_image_mat) {

    bool flag = 1;
    rs2::video_frame ir_frame = frame.get_infrared_frame(stereo_camera);
    const void* buffer = ir_frame.get_data();
    if (buffer) {
        int image_width = ir_frame.get_width();
        int image_height = ir_frame.get_height();
        int image_bits = ir_frame.get_bits_per_pixel();
        cv::Mat ir_image(cv::Size(image_width, image_height), CV_8UC1, (void*)buffer, cv::Mat::AUTO_STEP);
        ir_image_mat = ir_image;
    }
    else {
        std::cout << "Failed to get infrared image from capture" << std::endl;
        flag = 0;
    }
    return flag;
}

// get infrared image
bool Realsense::get_infrared_image() {
    bool flag = 1;
    bool flag_left = 1, flag_right = 1;
    ir_left_images.clear();
    ir_right_images.clear();
    cv::Mat ir_image_mat_left;
    cv::Mat ir_image_mat_right;
    for (auto& frame : frames) {
        flag_left = get_infrared(frame, 1, ir_image_mat_left);
        flag_right = get_infrared(frame, 2, ir_image_mat_right);
        ir_left_images.push_back(ir_image_mat_left);
        ir_right_images.push_back(ir_image_mat_right);
        if (!(flag_left && flag_right)) {
            flag = 0;
            break;
        }
    }
    return flag;
}

// save infrared image
void Realsense::save_infrared_image(const int& image_index) {
    int camera_index = 0;
    for (int index = 0; index < ir_left_images.size(); ++index) {
        //crop_image(color_mat);
        std::cout << "save infrared image at: " << image_index << std::endl;
        std::string image_depth_name_left = ".\\exportData\\ir\\l_ca" + std::to_string(camera_index)+ "_p" + std::to_string(image_index) + ".png";
        std::string image_depth_name_right = ".\\exportData\\ir\\r_ca" + std::to_string(camera_index) + "_p" + std::to_string(image_index) + ".png";
        cv::imwrite(image_depth_name_left, ir_left_images[index]);
        cv::imwrite(image_depth_name_right, ir_right_images[index]);
        camera_index++;
    }
    ir_left_images.clear();
    ir_right_images.clear();
}

// crop the images
void Realsense::crop_image(cv::Mat& showFrame) {
    int image_height, image_width;
    image_height = showFrame.rows;
    image_width = showFrame.cols;
    bool flag = FLAG_CROP;
    if (flag) {

        if (IMAGE_WIDTH < image_width && IMAGE_HEIGHT << image_height) {
            cv::Mat crop_image(showFrame, cv::Rect(IMAGE_START_X, IMAGE_START_Y, IMAGE_WIDTH, IMAGE_HEIGHT));
            showFrame = crop_image.clone();
        }
        else
            std::cout << "The size of objective image is out of range" << std::endl;
    }
}

// save raw data of infrared image, depth image.
void Realsense::save_raw_data(const int& index) {
    std::string file_ir_path_left = ".\\exportData\\raw_ir\\l_";
    std::string file_ir_path_right = ".\\exportData\\raw_ir\\r_";
    std::string file_depth_path = ".\\exportData\\raw_depth\\";
    std::string outputpath_depth, outputpath_ir_left, outputpath_ir_right;
    int camera_index = 0;
    for (auto& frame : frames) {
        outputpath_depth = file_depth_path + "ca" + std::to_string(camera_index) + "_" + "p" + std::to_string(index) + ".dat";
        outputpath_ir_left = file_ir_path_left + "ca" + std::to_string(camera_index) + "_" + "p" + std::to_string(index) + ".dat";
        outputpath_ir_right = file_ir_path_right + "ca" + std::to_string(camera_index) + "_" + "p" + std::to_string(index) + ".dat";
        std::ofstream depth_raw(outputpath_depth, std::ios::out);
        std::ofstream ir_left_raw(outputpath_ir_left, std::ios::out);
        std::ofstream ir_right_raw(outputpath_ir_right, std::ios::out);
        rs2::depth_frame depth_image = frame.get_depth_frame();
        const void* buffer_depth = depth_image.get_data();
        rs2::video_frame ir_frame_left = frame.get_infrared_frame(1);
        const void* buffer_left = ir_frame_left.get_data();
        rs2::video_frame ir_frame_right = frame.get_infrared_frame(2);
        const void* buffer_right = ir_frame_right.get_data();
        unsigned int image_width_depth = depth_image.get_width();
        unsigned int image_height_depth = depth_image.get_height();
        unsigned int image_width_ir_left = ir_frame_left.get_width();
        unsigned int image_height_ir_left = ir_frame_left.get_height();
        unsigned int image_width_ir_right = ir_frame_right.get_width();
        unsigned int image_height_ir_right = ir_frame_right.get_height();

        for (unsigned int i = 0; i < image_height_depth; ++i) {
            for (unsigned int j = 0; j < image_width_depth; ++j) {
                unsigned short image_value = *((uint16_t*)buffer_depth + i * image_width_depth + j);
                depth_raw << std::setw(5) << image_value << " ";
            }
            depth_raw << "\n";
        }

        for (unsigned int i = 0; i < image_height_ir_left; ++i) {
            for (unsigned int j = 0; j < image_width_ir_left; ++j) {
                unsigned short image_value = *((uint8_t*)buffer_left + i * image_width_ir_left + j);
                ir_left_raw << std::setw(7) << image_value << " ";
            }
            ir_left_raw << "\n";
        }

        for (unsigned int i = 0; i < image_height_ir_right; ++i) {
            for (unsigned int j = 0; j < image_width_ir_right; ++j) {
                unsigned short image_value = *((uint8_t*)buffer_right + i * image_width_ir_right + j);
                ir_right_raw << std::setw(7) << image_value << " ";
            }
            ir_right_raw << "\n";
        }
        camera_index++;
        depth_raw.close();
        ir_left_raw.close();
        ir_right_raw.close();
       
    }
}

void Realsense::videoShow() {
    int window_width = 640, window_height = 480;
    std::vector <std::string> window_name{ "Color live", "Depth live", "Left infrared live", "Right infrared live","Aligned depth live" };
    std::vector<std::vector<int>> window_position{ {0, 0},{ window_width * 1, 0},{ window_width * 2, 0}, {0, window_height * 1 + 30}, { window_width * 1, window_height * 1 + 30} };
    while (cv::waitKey(30) != 27) {
        std::vector<cv::Mat> image_to_show;
        retrieve_frame();
        get_color_image();
        get_depth_render_image();
        get_infrared_image();
        align_image();
        image_to_show.push_back(color_images[0]);
        image_to_show.push_back(depth_render_images[0]);
        image_to_show.push_back(ir_left_images[0]);
        image_to_show.push_back(ir_right_images[0]);
        image_to_show.push_back(after_align_images[0]);
        for (int i = 0; i < image_to_show.size(); ++i) {
            //crop_image(showFrame);
            cv::resizeWindow(window_name[i], window_width, window_height);
            cv::moveWindow(window_name[i], window_position[i][0], window_position[i][1]);
            /* cv::rectangle(image_to_show[i], cv::Rect(window_width / 2 - 90, window_height / 2 - 110, 180, 220), cv::Scalar(0, 255, 0), 2);
            cv::line(image_to_show[i], cv::Point(window_width / 2, window_height / 2 - 110), cv::Point(window_width / 2, window_height / 2 + 110), cv::Scalar(0, 255, 0), 1);
            cv::line(image_to_show[i], cv::Point(window_width / 2 - 90, window_height / 2), cv::Point(window_width / 2 + 90, window_height / 2), cv::Scalar(0, 255, 0), 1);
            */    
            cv::imshow(window_name[i], image_to_show[i]);
        }
    }
}

// Align one image to another.
void Realsense::align_image() {
    double beta = 0.0;
    beta = 1 - alpha;
    after_align_images.clear();
    aligned_depth_frame.clear();
    for (auto& frame : frames) {
        cv::Mat dst;
        rs2::frameset frame_update = alignment.process(frame);
        //rs2::frameset frame_update = frame;
        aligned_depth_frame.push_back(frame_update);
        rs2::depth_frame depth_image = frame_update.get_depth_frame();
        rs2::video_frame  colorized_depth = cmap.colorize(depth_image);
        const void* buffer_depth = colorized_depth.get_data();
        rs2::video_frame color_image = frame_update.get_color_frame();
        const void* buffer_color = color_image.get_data();

        if (buffer_depth && buffer_color) {
            unsigned int image_width_depth = colorized_depth.get_width();
            unsigned int image_height_depth = colorized_depth.get_height();
            cv::Mat depth_render_mat(cv::Size(image_width_depth, image_height_depth), CV_8UC3, (void*)buffer_depth, cv::Mat::AUTO_STEP);
            unsigned int image_width_color = color_image.get_width();
            unsigned int image_height_color = color_image.get_height();
            cv::Mat color_render_mat(cv::Size(image_width_color, image_height_color), CV_8UC3, (void*)buffer_color, cv::Mat::AUTO_STEP);
            addWeighted(depth_render_mat, alpha, color_render_mat, beta, 0.0, dst);
            after_align_images.push_back(dst);
        }

    }
}

// conver millisecond to time string.
void Realsense::gettime(int64 timestamp, std::string& Time)
{
    int64 milli = timestamp + (int64)8 * 60 * 60 * 1000; // conver to Beijing time zone. East 8
    auto mTime = std::chrono::milliseconds(milli);
    auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
    auto tt = std::chrono::system_clock::to_time_t(tp);
    struct tm buf;
    gmtime_s(&buf, &tt);
    int year = buf.tm_year + 1900;
    int month = buf.tm_mon + 1;
    int day = buf.tm_mday;
    int hour = buf.tm_hour;
    int min = buf.tm_min;
    int sec = buf.tm_sec;
    std::string year_s, month_s, day_s, hour_s, min_s, sec_s, millisec_s;
    year_s = std::to_string(year);
    if (month < 10)
         month_s = "0" + std::to_string(month);
    else
         month_s = std::to_string(month);
    if (day < 10)
        day_s = "0" + std::to_string(day);
    else
        day_s = std::to_string(day);
    if (hour < 10)
        hour_s = "0" + std::to_string(hour);
    else
        hour_s = std::to_string(hour);
    if (min < 10)
        min_s = "0" + std::to_string(min);
    else
        min_s = std::to_string(min);
    if (sec < 10)
        sec_s = "0" + std::to_string(sec);
    else
        sec_s = std::to_string(sec);

    std::string misec = std::to_string(milli);
    millisec_s = misec.substr(misec.size() - 3, 3);

    Time = year_s + month_s + day_s + hour_s + min_s + sec_s + millisec_s;
}

// Get point cloud from camera
void Realsense::get_points_cloud(const int& image_index, unsigned short& depth_value, const int& u, const int& v, bool ply_flag) {

    std::string outputpath_pointscloud;
    std::string file_pointscloud = ".\\exportData\\pointscloud\\";

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // Wait for the next set of frames from the camera
    rs2::frameset frame = aligned_depth_frame[0];
    // Get color frame
    rs2::video_frame color = frame.get_color_frame();
    // get depth 
    rs2::depth_frame depth = frame.get_depth_frame();

    const void* buffer_depth = depth.get_data();
    int image_width = depth.get_width();

    depth_value = *((uint16_t*)buffer_depth + v * image_width + u);

    //float dist_to_center = depth.get_distance(u, v);

    // Tell pointcloud object to map to this color frame
    pc.map_to(color);
    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    outputpath_pointscloud = file_pointscloud + "ply\\p" + std::to_string(image_index) + ".ply";
    if(ply_flag)
        points.export_to_ply(outputpath_pointscloud, color);

}
    
// save all point file
void Realsense::save_points_cloud(const int& image_index) {
    std::string file_pointscloud = ".\\exportData\\pointscloud\\";
    std::string outputpath_3d;
    outputpath_3d = file_pointscloud + "data\\p" + std::to_string(image_index) + ".dat";
    /* this segment actually prints the pointcloud */
    auto vertices = points.get_vertices();              // get vertices
    auto tex_coords = points.get_texture_coordinates(); // and texture coordinates

    std::ofstream threed_raw(outputpath_3d, std::ios::out);
    // Retrieve the coordinate x y z value (units: mm).
    int x = 0, y = 0, z = 0;
    for (unsigned int i = 0; i < INITIAL_HEIGHT; ++i) {
        for (unsigned int j = 0; j < INITIAL_WIDTH; ++j) {
            if (vertices[INITIAL_WIDTH * i + j].z >= 0 && abs(vertices[INITIAL_WIDTH * i + j].x) < 20 && abs(vertices[INITIAL_WIDTH * i + j].y) < 20) {
                x = int(vertices[INITIAL_WIDTH * i + j].x * 1000);
                y = int(vertices[INITIAL_WIDTH * i + j].y * 1000);
                z = int(vertices[INITIAL_WIDTH * i + j].z * 1000);

            }
            threed_raw << std::setprecision(5);
            threed_raw << "(" << x << ", " << y << ", " << z << ")" << " ";
           
        }
            threed_raw << "\n";
    }
        threed_raw.close();
}

// Obtain 3d coordinate value of the corresponding point in uv map(color frame)
void Realsense::get_point_3Dvalue(std::vector<int>& point_coordinate,const int& u, const int& v) {
    point_coordinate.clear();
    /* this segment actually prints the pointcloud */
    auto vertices = points.get_vertices();              // get vertices
    int x, y, z;
    //std::cout << "u: " << u << "v: " << v << std::endl;
    if (u > INITIAL_WIDTH-2 || (u < 1) || v > INITIAL_WIDTH-1 || v < 1)
        std::cout << "The querying map out of range" << std::endl;

    // use linear filter to reduce the noise
    std::vector<cv::Point> filter = { cv::Point(u-1,v-1), cv::Point(u,v - 1), cv::Point(u + 1,v - 1), 
                                      cv::Point(u - 1,v), cv::Point(u,v), cv::Point(u+1,v),
                                      cv::Point(u - 1,v+1), cv::Point(u,v+1), cv::Point(u + 1,v+1)};
    // Store the useful points
    std::vector<rs2::vertex> usefull_vertex;
    for (int point_index = 0; point_index < filter.size(); ++point_index) {
        // ignore the measured noise.
        int i = filter[point_index].x;
        int j = filter[point_index].y;

        if (vertices[INITIAL_WIDTH * j + i].z >= 0.39 && vertices[INITIAL_WIDTH * j + i].z <= 5 &&
            abs(vertices[INITIAL_WIDTH * j + i].x) < 5 && abs(vertices[INITIAL_WIDTH * j + i].y) < 5) {
            usefull_vertex.push_back(vertices[INITIAL_WIDTH * j + i]);
        }
    }
    //std::cout <<"usefull_vertex.size() : " << usefull_vertex.size() << std::endl;

    // use mean linear filter
    float x_sum = 0, y_sum=0, z_sum=0;
    for (int vertex_index = 0; vertex_index < usefull_vertex.size(); ++vertex_index) {
        x_sum += usefull_vertex[vertex_index].x;
        y_sum += usefull_vertex[vertex_index].y;
        z_sum += usefull_vertex[vertex_index].z;
    }
        x = int(x_sum / usefull_vertex.size() * 1000);
        y = int(y_sum / usefull_vertex.size() * 1000);
        z = int(z_sum / usefull_vertex.size() * 1000);
        point_coordinate.push_back(x);
        point_coordinate.push_back(y);
        point_coordinate.push_back(z);
}



// Get depth and color frame from the inner class
void Realsense::out_depth_color_to_main(cv::Mat& color_frame, cv::Mat& align_frame, std::string& time_stamp) {
    color_frame = color_images[0];
    align_frame = after_align_images[0];
    time_stamp = time_frame[0];
}


// detect connor points on the chessboarder
void Realsense::chess_connor_point_dection(cv::Mat& color_frame, std::vector<cv::Point2f>& pointbuf, const cv::Size& boardSize) {

    //cv::Mat frameGray;
    //cv::cvtColor(color_frame, frameGray, COLOR_BGR2GRAY);
    // if the chessboarder marker is small, use resize() to enlarge their geometries.
    cv::Size temp_size(1280, 720);
    cv::resize(color_frame, color_frame, temp_size, 0, 0, cv::INTER_CUBIC);

    bool found;
    // 
    found = findChessboardCornersSB(color_frame, boardSize, pointbuf, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
    if (found) {
        drawChessboardCorners(color_frame, boardSize, cv::Mat(pointbuf), found);

        // get points coordinates in uv map.
        //std::ofstream myfile;
        //string txtName = "corner" + to_string(i) + ".txt";
        //myfile.open(txtName, ios::out);
        //for (unsigned int j = 0; j < pointbuf.size(); j++)
        //{
        //	float x_pos = pointbuf.at(j).x;
        //	float y_pos = pointbuf.at(j).y;
        //	myfile << x_pos << " " << y_pos << std::endl;
        //}
        //myfile.close();

    }
}




 //save RGB images
//void Realsense::image_recog() {
//
//    std::string name_files = ".\\YOLO\\coco.names";
//    std::string modelConfiguration = ".\\YOLO\\yolov3.cfg";
//    std::string modelWeights = ".\\YOLO\\yolov3.weights";
//    std::vector<cv::Rect> boxes;
//    Objectdection image_dection(name_files, modelConfiguration, modelWeights);
//    cv::Mat frame = color_images[0];
//    image_dection.imageTo4D(frame);
//    image_dection.compute_image();
//    image_dection.postprocess(boxes);
//    for (auto& ele : boxes) {
//    	cout << "Height: " << ele.height << " Width: " << ele.width << " x: " << ele.x << " y: " << ele.y << endl;
//    }
//
//}



