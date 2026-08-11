#include<iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

enum class direction{
    to_depth,
    to_color
};

int myAlign(direction dir);
int AdvancedAlign();

int main(){

    direction dir=direction::to_color;
    myAlign(dir);
    return EXIT_SUCCESS;
}

int myAlign(direction dir)
{
    std::cout<<"RS Align"<<std::endl;

    rs2::context ctx;
    std::string serial;
    // 寻找是否存在同时支持彩色和深度流，并且序列号匹配的设备
    // if(!device_with_streams(ctx,{RS2_STREAM_COLOR,RS2_STREAM_DEPTH}, serial))
    //     return EXIT_SUCCESS;

    rs2::colorizer c;
    // texture depth_image, color_image; example中的辅助类。简化OpenGL纹理的管理和渲染。

    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_COLOR);
    // 返回相机实际启动了哪些流以及这些流的真实参数。
    rs2::pipeline_profile profile = pipe.start(cfg);

    // 创建align对象资源占用大，不适合在循环中创建
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    float alpha=0.5f;

    while(true){
        rs2::frameset frameset=pipe.wait_for_frames();

        if(dir==direction::to_depth){
            frameset=align_to_depth.process(frameset);  // align对检测到的帧集合进行处理，最终返回新的帧集合
        }
        else{
            frameset=align_to_color.process(frameset);
        }

        rs2::depth_frame depth=frameset.get_depth_frame();
        rs2::video_frame color=frameset.get_color_frame();
        rs2::video_frame colorized_depth=c.colorize(depth);

        cv::Mat color_rgb(
            cv::Size(color.get_width(),color.get_height()),
            CV_8UC3,
            (void*) color.get_data(),
            cv::Mat::AUTO_STEP
        );
        cv::Mat color_bgr;
        cv::cvtColor(color_rgb,color_bgr,cv::COLOR_RGB2BGR);

        cv::Mat depth_rgb(
            cv::Size(colorized_depth.get_width(),colorized_depth.get_height()),
            CV_8UC3,
            (void*)colorized_depth.get_data(),
            cv::Mat::AUTO_STEP
        );
        cv::Mat depth_bgr;
        cv::cvtColor(depth_rgb,depth_bgr,cv::COLOR_RGB2BGR);


        cv::Mat overlay;
        cv::addWeighted(color_bgr, 0.5, depth_bgr, 0.5, 0, overlay);
        cv::imshow("Aligned Overlay",overlay);

        char key=cv::waitKey(1);
        if(key=='q'||key==27)
            break;
    }
    cv::destroyAllWindows();
    
    return EXIT_SUCCESS;
}

int AdvancedAlign(){
    return 0;
}