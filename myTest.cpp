// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019-24 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp> // RealSense Cross Platform API
#include <iostream>
#include <opencv2/opencv.hpp>

#include <thread>
#include <chrono>

using namespace std;

/*
彩色相机帧尺寸参数：1280 720 8UC3
深度相机帧尺寸参数：848 480 16UC1
*/

int test_cv();
void show_color_frame(const rs2::frameset& frames);
void show_depth_frame(const rs2::depth_frame& depth_frame, const rs2::colorizer& color_map);
void get_center_depth(const rs2::depth_frame& depth);
void set_sensor(const rs2::pipeline_profile& selection);
void OCC(rs2::pipeline_profile& prof);


// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char * argv[]) try
{
    rs2::pipeline p;

    // 彩色深度图绘制
    rs2::colorizer color_map;
    rs2::rates_printer printer;
    rs2::config cfg;

    rs2::pointcloud ps;
    rs2::points points;

    // Configuration
    cfg.enable_all_streams();
    //cfg.enable_stream( RS2_STREAM_DEPTH,256,144,RS2_FORMAT_Z16,90);

    // Configure and start the pipeline
    rs2::pipeline_profile selection = p.start(cfg);

    // set_sensor(selection);

    // 相机数据标定
    // p.wait_for_frames(); // 必须输出第一帧，realsense才可以开始标定。
    // OCC(selection);

    while (true)
    {
        // Block program until frames arrive
        // 方法链(Method Chaining)，流式接口
        // 连学调用对象的成员函数，将多个操作串成一个表达式
        rs2::frameset frames = p.wait_for_frames().apply_filter(printer);
        rs2::depth_frame depth = frames.get_depth_frame();
        if (!depth) {
            continue;
        }

        show_depth_frame(depth, color_map);
        
        char key=cv::waitKey(1);
        if(key=='q'||key==27)
            break;
    }
    p.stop();
    cv::destroyAllWindows();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\r    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

int test_cv(){
     cv::Mat image=cv::imread("../CVTest.jpeg");
    if(image.empty()){
        std::cout<<"None!"<<std::endl;
        return -1;
    }
    cv::imshow("Display Image",image);
    int key=cv::waitKey(0);
    cv::destroyAllWindows();

    if(key=='s')
        return 0;
    return -1;
}

void show_color_frame(const rs2::frameset& frames){
    rs2::video_frame color_frame=frames.get_color_frame();

    if(!color_frame)
        return;

    auto width=color_frame.get_width();
    auto height=color_frame.get_height();

    if(color_frame){
        cv::Mat color_mat(cv::Size(width,height),CV_8UC3,(void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
        cv::Mat bgr_mat;
        cv::cvtColor(color_mat,bgr_mat,cv::COLOR_RGB2BGR);
        cv::imshow("D435i Frame",bgr_mat);
    }
    return;
}

void show_depth_frame(const rs2::depth_frame& depth_frame, const rs2::colorizer& color_map){
    if(!depth_frame) return;

    rs2::video_frame colorized_depth = color_map.colorize(depth_frame);
    if(!colorized_depth) return;

    //std::cout<<"\r"<<colorized_depth.get_width()<<' '<<colorized_depth.get_height()<<std::flush;

    cv::Mat depth_rgb(cv::Size(colorized_depth.get_width(), colorized_depth.get_height()),
                      CV_8UC3,
                      (void*)colorized_depth.get_data(),
                      cv::Mat::AUTO_STEP);
    cv::Mat depth_bgr;
    cv::cvtColor(depth_rgb, depth_bgr, cv::COLOR_RGB2BGR);
    cv::imshow("Depth (Colored)", depth_bgr);
}

void get_center_depth(const rs2::depth_frame& depth){
        // Get the depth frame's dimensions
        auto width = depth.get_width();
        auto height = depth.get_height();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);
 
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \r    ";

}

void set_sensor(const rs2::pipeline_profile& selection){
    // 红外摄像头的设置和启用
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    // enable emiter and set the power
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        //depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }
}

void OCC(rs2::pipeline_profile& prof){
    std::stringstream ss;
    ss  << "{" 
        << "\n \"calib type\":"<<0
        << ",\n \"speed\":" <<2
        << ",\n \"scan parameter\":" <<0
        << "\n}";
    std::string json = ss.str();
    std::cout << "Starting OCC with configuration:\n"<<json<<std::endl;

    rs2::device dev=prof.get_device();
    rs2::auto_calibrated_device cal_dev=dev.as<rs2::auto_calibrated_device>();

    // run calibration
    float health;
    int timeout_ms=9000;
    rs2::calibration_table res=cal_dev.run_on_chip_calibration(json,&health,[&](const float progress){
        std::cout<<"progress = "<<progress<<"%"<<std::endl;
    }, timeout_ms);

    std::cout<<"Completed successfullu"<<std::endl;

    std::cout<<std::endl<<"Keep results? Yes/No"<<std::endl;
    std::string keep;
    std::cin>>keep;
    for(auto & c:keep)
        c=tolower(c);
    if(keep=="y"||keep=="yes")
    {
        cal_dev.set_calibration_table(res);
        cal_dev.write_calibration();
        std::cout<<"Result saved to flush"<<std::endl;
    }
    else
    {
        std::cout<<"Results not save"<<std::endl;
    }
}
