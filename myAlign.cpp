#include<iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

enum class direction{
    to_depth,
    to_color
};

int myAlign(direction dir);
int AdvancedAlign();

// 辅助函数
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current,const std::vector<rs2::stream_profile>& prev);
void remove_background(rs2::video_frame& other, const rs2::depth_frame depth_frame, float depth_scale, float clipping_dist);

int main(){

    direction dir=direction::to_color;
    // myAlign(dir);
    AdvancedAlign();
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
    /*
        启动后，显示过滤深度大于thresdhold的背景，
        提供一个交互滑块，控制threshold的大小
        提供一个深度图，显示整体深度图像的深度分布，同时显示一个标识线显示当前threshold对应深度位置。
    */

    std::stringstream ss;
    ss  << "{" 
        << "\n \"calib type\":"<<0
        << ",\n \"speed\":" <<2
        << ",\n \"scan parameter\":" <<0
        << "\n}";
    std::string json = ss.str();
    rs2::context ctx(json);

    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    rs2::pipeline_profile prof=pipe.start(cfg);

    rs2::colorizer c;
    // 判断有无彩色流，无彩色流与其它流进行对齐
    rs2_stream align_to = find_stream_to_align(prof.get_streams());
    rs2::align align(align_to);

    float depth_scale = get_depth_scale(prof.get_device());
    float threshold=0.75f;

    while(true){
        rs2::frameset frames=pipe.wait_for_frames();
        // 如果设备错误或者断联，wait_for_frame可能会切换设备
        // rs2::align将深度与其它流对齐，需要确保其它流不改变
        if(profile_changed(pipe.get_active_profile().get_streams(),prof.get_streams())){
            prof=pipe.get_active_profile();
            align_to=find_stream_to_align(prof.get_streams());
            align=rs2::align(align_to);
            depth_scale=get_depth_scale(prof.get_device());
        }
        
        rs2::frameset processed = align.process(frames);
        rs2::video_frame other_frame =processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        if(!aligned_depth_frame || !other_frame)
            continue;

        remove_background(other_frame,aligned_depth_frame,depth_scale,threshold);

        cv::Mat img_rgb(cv::Size(other_frame.get_width(),other_frame.get_height()),CV_8UC3,(void*)other_frame.get_data(),cv::Mat::AUTO_STEP);
        cv::Mat img_bgr;
        cv::cvtColor(img_rgb,img_bgr,cv::COLOR_RGB2BGR);

        cv::imshow("遮罩50%深度",img_bgr);

        char key=cv::waitKey(1);
        if(key=='q'||key==27)
            break;
    }
    cv::destroyAllWindows();

    return 0;
}

// 获取缩放因子，将深度像素转换成物理距离
float get_depth_scale(rs2::device dev){
    // 引用式遍历
    for(rs2::sensor& sensor : dev.query_sensors())
    {
        // 声明变量，同时判断其是否有效
        if(rs2::depth_sensor dpt =sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams){
    // 给定多种流的向量，将深度流与其它流进行对齐。
    // 优先选择彩色流，如果没有彩色流，则将深度流与其它流进行对齐。
    rs2_stream align_to=RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;

    for(rs2::stream_profile sp : streams){
        rs2_stream profile_stream =sp.stream_type();
        if(profile_stream!=RS2_STREAM_DEPTH){
            if(!color_stream_found)
                align_to=profile_stream;
            if(profile_stream==RS2_STREAM_COLOR){
                color_stream_found=true;
            }
        }
        else{
            depth_stream_found=true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");
    if(align_to==RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");
    
    return align_to;
}

// 检测RealSense设备流配置是否发生变化
bool profile_changed(const std::vector<rs2::stream_profile>& current,const std::vector<rs2::stream_profile>& prev){
    for(auto& sp :prev)
    {
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp){return sp.unique_id()==current_sp.unique_id();});
        if(itr==std::end(current)){
            return true;
        }
    }
    return false;
}

// 依据深度图像对其它流图像背景进行遮罩操作，并进行修改。
void remove_background(rs2::video_frame& other_frame, const rs2::depth_frame depth_frame, float depth_scale, float clipping_dist){
    // uint16_t类型确保数据长度为16位
    const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    uint8_t* p_other_frame =reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    int width=other_frame.get_width();
    int height=other_frame.get_height();
    int other_bpp=other_frame.get_bytes_per_pixel();    // 自动获取帧像素字节位数

    #pragma omp parallel for schedule(dynamic)
    for(int y=0;y<height;y++){
        auto depth_pixel_index=y*width;

        for(int x=0;x<width;x++,++depth_pixel_index){
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

            if(pixels_distance<=0.f||pixels_distance>clipping_dist){
                auto offset=depth_pixel_index * other_bpp;
                std::memset(&p_other_frame[offset],0x99,other_bpp);
            }
        }
    }
}