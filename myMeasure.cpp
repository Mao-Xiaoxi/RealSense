#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/gl.h>
#include <FL/Fl_Widget.H>
#include <FL/fl_draw.H>

#include "auxiliary.h"

// This example will require several standard data-structures and algorithms:
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cstring>
#include <vector>
#include <csignal>
#include <algorithm>
#include <iomanip>
#include <sstream>

// 定义别名
using pixel = std::pair<int,int>;

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_stop_signal(int)
{
    g_stop_requested = 1;
}

float dist_3d(const rs2::depth_frame& frame, pixel u, pixel v);

class MyGlWindow : public Fl_Widget {
public:
    MyGlWindow(int x, int y, int w, int h, const char* l = 0)
        : Fl_Widget(x, y, w, h, l) {}

    // 由外部调用来更新内部数据（从队列取帧）
    void update_frames(const rs2::frameset& new_frames) {
        if (!new_frames) return;

        current_depth = new_frames.get_depth_frame();
        auto colorized_depth = new_frames.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);
        auto color = new_frames.get_color_frame();

        bool copied = false;
        if (colorized_depth && color) {
            copied = copy_overlay(colorized_depth.as<rs2::video_frame>(),
                                  color.as<rs2::video_frame>());
        }
        if (!copied && colorized_depth) {
            copied = copy_frame(colorized_depth.as<rs2::video_frame>());
        }
        if (!copied && color) {
            copied = copy_frame(color.as<rs2::video_frame>());
        }

        if (copied) {
            static int update_count = 0;
            if (++update_count % 30 == 1) {
                std::cout << "[ui] copied drawable frame #" << update_count
                          << " (" << image_width << "x" << image_height
                          << ", channels=" << image_channels << ")" << std::endl;
            }
            redraw(); // 请求重绘
        }
    }

    int handle(int event) override {
        if (event == FL_PUSH && Fl::event_button() == FL_LEFT_MOUSE &&
            image_width > 0 && image_height > 0) {
            if (measure_points.size() >= 2) {
                measure_points.clear();
            }
            measure_points.push_back(window_to_image(Fl::event_x(), Fl::event_y()));
            redraw();
            return 1;
        }

        if (event == FL_PUSH && Fl::event_button() == FL_RIGHT_MOUSE) {
            measure_points.clear();
            redraw();
            return 1;
        }

        return Fl_Widget::handle(event);
    }

protected:
    void draw() override {
        fl_color(15, 15, 20);
        fl_rectf(x(), y(), w(), h());

        if (image_data.empty()) {
            static bool printed_no_frame = false;
            if (!printed_no_frame) {
                std::cout << "[draw] window is drawing, but no drawable image has arrived yet"
                          << std::endl;
                printed_no_frame = true;
            }
            return;
        }

        static int draw_count = 0;
        if (++draw_count % 30 == 1) {
            std::cout << "[draw] native rendering image #" << draw_count
                      << " (" << image_width << "x" << image_height
                      << ", channels=" << image_channels << ")" << std::endl;
        }

        if (image_width == w() && image_height == h()) {
            fl_draw_image(image_data.data(), x(), y(), image_width, image_height, image_channels);
        } else {
            resized_image.resize(static_cast<size_t>(w()) * h() * image_channels);
            cv::Mat src(image_height, image_width, CV_8UC3, image_data.data());
            cv::Mat dst(h(), w(), CV_8UC3, resized_image.data());
            cv::resize(src, dst, dst.size(), 0, 0, cv::INTER_LINEAR);
            fl_draw_image(resized_image.data(), x(), y(), w(), h(), image_channels);
        }

        draw_measurement();
    }

private:
    std::vector<unsigned char> image_data;
    std::vector<unsigned char> resized_image;
    std::vector<pixel> measure_points;
    rs2::frame current_depth;
    int image_width = 0;
    int image_height = 0;
    int image_channels = 3;
    // 状态（端点、鼠标等）—— 可放在此处或作为外部全局
    // state app_state;

    bool copy_frame(const rs2::video_frame& vf) {
        if (!vf) return false;

        auto format = vf.get_profile().format();

        if (format == RS2_FORMAT_RGB8 || format == RS2_FORMAT_BGR8) {
            image_channels = 3;
        } else if (format == RS2_FORMAT_RGBA8 || format == RS2_FORMAT_BGRA8) {
            image_channels = 3;
        } else if (format == RS2_FORMAT_Y8) {
            image_channels = 3;
        } else {
            std::cout << "[ui] unsupported video frame format: "
                      << static_cast<int>(format) << std::endl;
            return false;
        }

        image_width = vf.get_width();
        image_height = vf.get_height();
        image_data.resize(static_cast<size_t>(image_width) * image_height * image_channels);

        const auto* src = static_cast<const unsigned char*>(vf.get_data());
        const int src_stride = vf.get_stride_in_bytes();
        const int dst_stride = image_width * image_channels;

        if (format == RS2_FORMAT_RGB8) {
            for (int y = 0; y < image_height; ++y) {
                std::memcpy(image_data.data() + static_cast<size_t>(y) * dst_stride,
                            src + static_cast<size_t>(y) * src_stride,
                            dst_stride);
            }
            return true;
        }

        if (format == RS2_FORMAT_BGR8) {
            for (int y = 0; y < image_height; ++y) {
                const auto* src_row = src + static_cast<size_t>(y) * src_stride;
                auto* dst_row = image_data.data() + static_cast<size_t>(y) * dst_stride;
                for (int x = 0; x < image_width; ++x) {
                    dst_row[x * 3 + 0] = src_row[x * 3 + 2];
                    dst_row[x * 3 + 1] = src_row[x * 3 + 1];
                    dst_row[x * 3 + 2] = src_row[x * 3 + 0];
                }
            }
            return true;
        }

        if (format == RS2_FORMAT_RGBA8) {
            for (int y = 0; y < image_height; ++y) {
                const auto* src_row = src + static_cast<size_t>(y) * src_stride;
                auto* dst_row = image_data.data() + static_cast<size_t>(y) * dst_stride;
                for (int x = 0; x < image_width; ++x) {
                    dst_row[x * 3 + 0] = src_row[x * 4 + 0];
                    dst_row[x * 3 + 1] = src_row[x * 4 + 1];
                    dst_row[x * 3 + 2] = src_row[x * 4 + 2];
                }
            }
            return true;
        }

        if (format == RS2_FORMAT_BGRA8) {
            for (int y = 0; y < image_height; ++y) {
                const auto* src_row = src + static_cast<size_t>(y) * src_stride;
                auto* dst_row = image_data.data() + static_cast<size_t>(y) * dst_stride;
                for (int x = 0; x < image_width; ++x) {
                    dst_row[x * 3 + 0] = src_row[x * 4 + 2];
                    dst_row[x * 3 + 1] = src_row[x * 4 + 1];
                    dst_row[x * 3 + 2] = src_row[x * 4 + 0];
                }
            }
            return true;
        }

        for (int y = 0; y < image_height; ++y) {
            const auto* src_row = src + static_cast<size_t>(y) * src_stride;
            auto* dst_row = image_data.data() + static_cast<size_t>(y) * dst_stride;
            for (int x = 0; x < image_width; ++x) {
                dst_row[x * 3 + 0] = src_row[x];
                dst_row[x * 3 + 1] = src_row[x];
                dst_row[x * 3 + 2] = src_row[x];
            }
        }

        return true;
    }

    bool copy_overlay(const rs2::video_frame& colorized_depth,
                      const rs2::video_frame& color) {
        std::vector<unsigned char> depth_rgb;
        std::vector<unsigned char> color_rgb;
        int depth_width = 0;
        int depth_height = 0;
        int color_width = 0;
        int color_height = 0;

        if (!copy_frame_to_rgb(colorized_depth, depth_rgb, depth_width, depth_height) ||
            !copy_frame_to_rgb(color, color_rgb, color_width, color_height)) {
            return false;
        }

        cv::Mat depth_mat(depth_height, depth_width, CV_8UC3, depth_rgb.data());
        cv::Mat color_mat(color_height, color_width, CV_8UC3, color_rgb.data());
        if (color_mat.size() != depth_mat.size()) {
            cv::resize(color_mat, color_mat, depth_mat.size(), 0, 0, cv::INTER_LINEAR);
        }

        image_width = depth_width;
        image_height = depth_height;
        image_channels = 3;
        image_data.resize(static_cast<size_t>(image_width) * image_height * image_channels);

        cv::Mat overlay(image_height, image_width, CV_8UC3, image_data.data());
        cv::addWeighted(color_mat, 0.45, depth_mat, 0.55, 0.0, overlay);
        return true;
    }

    bool copy_frame_to_rgb(const rs2::video_frame& vf,
                           std::vector<unsigned char>& output,
                           int& output_width,
                           int& output_height) {
        if (!vf) return false;

        auto format = vf.get_profile().format();
        output_width = vf.get_width();
        output_height = vf.get_height();
        output.resize(static_cast<size_t>(output_width) * output_height * 3);

        const auto* src = static_cast<const unsigned char*>(vf.get_data());
        const int src_stride = vf.get_stride_in_bytes();
        const int dst_stride = output_width * 3;

        if (format == RS2_FORMAT_RGB8) {
            for (int y = 0; y < output_height; ++y) {
                std::memcpy(output.data() + static_cast<size_t>(y) * dst_stride,
                            src + static_cast<size_t>(y) * src_stride,
                            dst_stride);
            }
            return true;
        }

        if (format == RS2_FORMAT_BGR8) {
            for (int y = 0; y < output_height; ++y) {
                const auto* src_row = src + static_cast<size_t>(y) * src_stride;
                auto* dst_row = output.data() + static_cast<size_t>(y) * dst_stride;
                for (int x = 0; x < output_width; ++x) {
                    dst_row[x * 3 + 0] = src_row[x * 3 + 2];
                    dst_row[x * 3 + 1] = src_row[x * 3 + 1];
                    dst_row[x * 3 + 2] = src_row[x * 3 + 0];
                }
            }
            return true;
        }

        if (format == RS2_FORMAT_RGBA8) {
            for (int y = 0; y < output_height; ++y) {
                const auto* src_row = src + static_cast<size_t>(y) * src_stride;
                auto* dst_row = output.data() + static_cast<size_t>(y) * dst_stride;
                for (int x = 0; x < output_width; ++x) {
                    dst_row[x * 3 + 0] = src_row[x * 4 + 0];
                    dst_row[x * 3 + 1] = src_row[x * 4 + 1];
                    dst_row[x * 3 + 2] = src_row[x * 4 + 2];
                }
            }
            return true;
        }

        if (format == RS2_FORMAT_BGRA8) {
            for (int y = 0; y < output_height; ++y) {
                const auto* src_row = src + static_cast<size_t>(y) * src_stride;
                auto* dst_row = output.data() + static_cast<size_t>(y) * dst_stride;
                for (int x = 0; x < output_width; ++x) {
                    dst_row[x * 3 + 0] = src_row[x * 4 + 2];
                    dst_row[x * 3 + 1] = src_row[x * 4 + 1];
                    dst_row[x * 3 + 2] = src_row[x * 4 + 0];
                }
            }
            return true;
        }

        if (format == RS2_FORMAT_Y8) {
            for (int y = 0; y < output_height; ++y) {
                const auto* src_row = src + static_cast<size_t>(y) * src_stride;
                auto* dst_row = output.data() + static_cast<size_t>(y) * dst_stride;
                for (int x = 0; x < output_width; ++x) {
                    dst_row[x * 3 + 0] = src_row[x];
                    dst_row[x * 3 + 1] = src_row[x];
                    dst_row[x * 3 + 2] = src_row[x];
                }
            }
            return true;
        }

        std::cout << "[ui] unsupported video frame format for overlay: "
                  << static_cast<int>(format) << std::endl;
        return false;
    }

    pixel window_to_image(int window_x, int window_y) const {
        int local_x = std::clamp(window_x - x(), 0, std::max(0, w() - 1));
        int local_y = std::clamp(window_y - y(), 0, std::max(0, h() - 1));

        int image_x = local_x * image_width / std::max(1, w());
        int image_y = local_y * image_height / std::max(1, h());

        image_x = std::clamp(image_x, 0, std::max(0, image_width - 1));
        image_y = std::clamp(image_y, 0, std::max(0, image_height - 1));
        return {image_x, image_y};
    }

    pixel image_to_window(pixel p) const {
        int window_x = x() + p.first * std::max(1, w()) / std::max(1, image_width);
        int window_y = y() + p.second * std::max(1, h()) / std::max(1, image_height);
        return {window_x, window_y};
    }

    void draw_point_marker(pixel p) const {
        auto wp = image_to_window(p);
        fl_color(255, 255, 255);
        fl_pie(wp.first - 5, wp.second - 5, 10, 10, 0, 360);
        fl_color(20, 20, 20);
        fl_pie(wp.first - 2, wp.second - 2, 4, 4, 0, 360);
    }

    void draw_measurement() const {
        for (auto p : measure_points) {
            draw_point_marker(p);
        }

        if (measure_points.size() != 2 || !current_depth) {
            return;
        }

        auto p0 = image_to_window(measure_points[0]);
        auto p1 = image_to_window(measure_points[1]);

        fl_color(255, 255, 255);
        fl_line_style(FL_SOLID, 2);
        fl_line(p0.first, p0.second, p1.first, p1.second);
        fl_line_style(0);

        auto depth = current_depth.as<rs2::depth_frame>();
        if (!depth) {
            return;
        }

        float distance = dist_3d(depth, measure_points[0], measure_points[1]);
        std::ostringstream label;
        label << std::fixed << std::setprecision(3) << distance << " m";

        int label_x = (p0.first + p1.first) / 2 + 8;
        int label_y = (p0.second + p1.second) / 2 - 8;
        fl_color(0, 0, 0);
        fl_rectf(label_x - 4, label_y - 16, 86, 22);
        fl_color(255, 255, 255);
        fl_draw(label.str().c_str(), label_x, label_y);
    }
};

struct AppData {
    MyGlWindow* glWindow;
    rs2::pipeline* pipe;
    rs2::colorizer* colorMap;
    rs2::align* alignTo;
    rs2::decimation_filter* decimation;
    rs2::disparity_transform* depthToDisparity;
    rs2::spatial_filter* spatial;
    rs2::temporal_filter* temporal;
    rs2::disparity_transform* disparityToDepth;
    bool enableDecimation;
    bool enableDisparity;
    bool enableSpatial;
    bool enableTemporal;
};

int main(int argc,char **argv) try
{
    std::cout << "[main] myMeasure startup diagnostics v11" << std::endl;
    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);

    auto settings = CLI("myMeasure").process(argc,argv);
    rs2::context ctx;
    std::string serial = settings.get("serial");
    bool enable_decimation = settings.get("decimation") == "true" ||
                             settings.get("decimation") == "1";
    bool enable_spatial = settings.get("spatial") == "true" ||
                          settings.get("spatial") == "1";
    bool enable_temporal = settings.get("temporal") == "true" ||
                           settings.get("temporal") == "1";
    bool enable_disparity = settings.get("disparity") == "true" ||
                            settings.get("disparity") == "1" ||
                            enable_spatial || enable_temporal;
    if (!serial.empty()) {
        std::cout << "[main] requested device serial: " << serial << std::endl;
    } else {
        std::cout << "[main] using first available RealSense device" << std::endl;
    }
    std::cout << "[main] decimation: "
              << (enable_decimation ? "enabled" : "disabled") << std::endl;
    std::cout << "[main] disparity: "
              << (enable_disparity ? "enabled" : "disabled") << std::endl;
    std::cout << "[main] spatial: "
              << (enable_spatial ? "enabled" : "disabled") << std::endl;
    std::cout << "[main] temporal: "
              << (enable_temporal ? "enabled" : "disabled") << std::endl;

    rs2::colorizer color_map;
    // 0 = Jet 彩色色带；2/3 更接近黑白色带。
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 0.f);

    // 配置降采样滤波器，降低深度图的分辨率来提高数据处理的速度
    rs2::decimation_filter dec;
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);

    // 分别创建视差和深度相互转换的变换器
    // 主要目的：提升后处理滤波（比如空间滤波和时间滤波）的效果。
    // 在深度域中，测量误差的一个主要来源是视差误差，它的影响会随着距离的增加而放大。因此将深度域转换成视差域能够更有效地平滑噪声。
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth(false);

    // 边缘保留的平滑滤波器，平衡去噪和保留细节
    rs2::spatial_filter spat;
    // 空洞填充，0-5，不填充-激进填充
    spat.set_option(RS2_OPTION_HOLES_FILL, 5);
    // 利用历史帧信息来减少深度值的抖动。
    rs2::temporal_filter temp;

    rs2::align align_to(RS2_STREAM_DEPTH);
    rs2::pipeline pipe(ctx);

    rs2::config cfg;
    if(!serial.empty())
        cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    std::cout << "[main] starting pipeline" << std::endl;
    auto profile = pipe.start(cfg);
    std::cout << "[main] pipeline started" << std::endl;

    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    std::cout << "[main] depth stream profile: "
              << stream.width() << "x" << stream.height()
              << " @" << stream.fps() << "fps" << std::endl;

    // 创建官方示例OpenGL等效窗口
    Fl_Window window(stream.width(), stream.height(), "RealSense MyMeasure");
    MyGlWindow glWindow(0, 0, stream.width(), stream.height());
    glWindow.show();
    window.end();
    window.show(argc, argv);
    std::cout << "[main] FLTK window shown" << std::endl;

    AppData app_data{
        &glWindow,
        &pipe,
        &color_map,
        &align_to,
        &dec,
        &depth2disparity,
        &spat,
        &temp,
        &disparity2depth,
        enable_decimation,
        enable_disparity,
        enable_spatial,
        enable_temporal
    };

    Fl::add_idle([](void* data) {
        auto* app = static_cast<AppData*>(data);

        if (g_stop_requested) {
            if (auto* win = app->glWindow->window()) {
                win->hide();
            }
            return;
        }

        rs2::frameset frames;
        try {
            if (app->pipe->poll_for_frames(&frames)) {
                frames = frames.apply_filter(*app->alignTo);
                if (app->enableDecimation) {
                    frames = frames.apply_filter(*app->decimation);
                }
                if (app->enableDisparity) {
                    frames = frames.apply_filter(*app->depthToDisparity);
                }
                if (app->enableSpatial) {
                    frames = frames.apply_filter(*app->spatial);
                }
                if (app->enableTemporal) {
                    frames = frames.apply_filter(*app->temporal);
                }
                if (app->enableDisparity) {
                    frames = frames.apply_filter(*app->disparityToDepth);
                }
                frames = frames.apply_filter(*app->colorMap);

                static int frame_count = 0;
                if (++frame_count % 30 == 1) {
                    std::cout << "[idle] processed frameset #" << frame_count
                              << " [decimation=" << app->enableDecimation
                              << ", disparity=" << app->enableDisparity
                              << ", spatial=" << app->enableSpatial
                              << ", temporal=" << app->enableTemporal
                              << "]" << std::endl;
                }
                app->glWindow->update_frames(frames);
            } else {
                static auto last_report = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (now - last_report > std::chrono::seconds(2)) {
                    std::cout << "[idle] no frame available yet" << std::endl;
                    last_report = now;
                }
            }
        }
        catch (const rs2::error& e) {
            std::cerr << "[idle] RealSense polling error: "
                      << e.what() << std::endl;
        }
    }, &app_data);

    std::cout << "[main] entering FLTK event loop" << std::endl;
    Fl::run();
    std::cout << "[main] FLTK event loop exited" << std::endl;

    std::cout << "[main] stopping RealSense pipeline" << std::endl;
    try {
        pipe.stop();
    }
    catch (const rs2::error& e) {
        std::cerr << "[main] pipe.stop warning: " << e.what() << std::endl;
    }
    std::cout << "[main] shutdown complete" << std::endl;
    return 0;
}
catch(const rs2::error & e){
    std::cerr<<"RealSense error calling"<<e.get_failed_function()<<"("<<e.get_failed_args()<<")"<<e.what()<<std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

float dist_3d(const rs2::depth_frame& frame, pixel u, pixel v)
{
    float upixel[2]; // From pixel
    float upoint[3]; // From point (in 3D)

    float vpixel[2]; // To pixel
    float vpoint[3]; // To point (in 3D)

    // Copy pixels into the arrays (to match rsutil signatures)
    upixel[0] = static_cast<float>(u.first);
    upixel[1] = static_cast<float>(u.second);
    vpixel[0] = static_cast<float>(v.first);
    vpixel[1] = static_cast<float>(v.second);

    // Query the frame for distance
    // Note: this can be optimized
    // It is not recommended to issue an API call for each pixel
    // (since the compiler can't inline these)
    // However, in this example it is not one of the bottlenecks
    auto udist = frame.get_distance(static_cast<int>(upixel[0]), static_cast<int>(upixel[1]));
    auto vdist = frame.get_distance(static_cast<int>(vpixel[0]), static_cast<int>(vpixel[1]));

    // Deproject from pixel to point in 3D
    rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
    rs2_deproject_pixel_to_point(vpoint, &intr, vpixel, vdist);

    // Calculate euclidean distance between the two points
    return sqrt(pow(upoint[0] - vpoint[0], 2.f) +
                pow(upoint[1] - vpoint[1], 2.f) +
                pow(upoint[2] - vpoint[2], 2.f));
}
