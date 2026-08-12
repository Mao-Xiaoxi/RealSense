#include "auxiliary.h"

// 解析命令行参数，返回自身引用以便链式调用
CLI& CLI::process(int argc, char* argv[]) {
    // store the original parameter for help information.
    program_name_=argv[0];
    raw_args_.assign(argv+1,argv + argc);

    for(int i=1;i<argc;++i){
        std::string arg=argv[i];

        if(arg=="--help"){
            print_help();
            exit(0);
        }
        else if(arg=="--serial"){
            if(i+1<argc){
                options_["serial"] = argv[++i];
            }else{
                std::cerr<<"Error: --serial requires a value"<<std::endl;
                exit(1);
            }
        }
        else if(arg=="--nodds"){
            options_["nodds"]="true";
        }
        else if(arg.find("--")==0){
            // deal with the form: --key=value
            auto pos=arg.find('=');
            if(pos!=std::string::npos){
                std::string key = arg.substr(2,pos-2);
                std::string value = arg.substr(pos+1);
                options_[key] = value;
            }else{
                std::cerr << "Warning: unknown option "<<arg<<std::endl;
            }
        }
    }

    return *this;
}

// const的意义
void CLI::print_help() const {
    std::cout << "Usage: " << program_name_ << " [options]\n";
    std::cout << description_ << "\n\n";
    std::cout << "Options:\n";
    std::cout << "  --help               Show this help message\n";
    std::cout << "  --serial <SN>        Connect to device with serial number\n";
    std::cout << "  --log-level <level>  Set log level (info, debug, warn, error)\n";
    std::cout << "  --nodds              Disable DDS communication\n";
    // 可扩展更多选项
}

// 返回json格式字符串
std::string CLI::dump() const{
    std::ostringstream oss;
    oss<<"{";
    bool first =true;
    for(const auto& pair : options_){
        if (!first) oss << ",";
        first = false;
        oss << "\"" << pair.first << "\":\"" << pair.second << "\"";
    }
    oss<<"}";
    return oss.str();
  }


  /**
 * 查找支持所有指定流类型的设备，并可选择匹配序列号。
 * 
 * @param ctx     RealSense 上下文
 * @param streams 所需流的列表（如 {RS2_STREAM_COLOR, RS2_STREAM_DEPTH}）
 * @param serial  输入/输出参数：若传入非空，则仅匹配该序列号的设备；
 *                若为空，则返回第一个匹配设备的序列号。
 * @return        若找到匹配设备，返回 true，并将 serial 设置为该设备的序列号；
 *                否则返回 false，serial 保持不变。
 */
bool device_with_streams(rs2::context& ctx, std::initializer_list<rs2_stream> streams, std::string& serial)
{
    // 获取所有已连接的设备
    rs2::device_list devices = ctx.query_devices();

    // 遍历设备
    for (const rs2::device& dev : devices) {
        // 获取当前设备的序列号
        std::string dev_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        // 如果调用者指定了要匹配的序列号，且与当前设备不符，跳过
        if (!serial.empty() && serial != dev_serial) {
            continue;
        }

        // 收集当前设备所有传感器支持的流类型
        std::set<rs2_stream> supported_streams;
        auto sensors = dev.query_sensors();
        for (const rs2::sensor& sensor : sensors) {
            auto profiles = sensor.get_stream_profiles();
            for (const rs2::stream_profile& prof : profiles) {
                supported_streams.insert(prof.stream_type());
            }
        }

        // 检查是否支持所有请求的流
        bool all_supported = true;
        for (rs2_stream req_stream : streams) {
            if (supported_streams.find(req_stream) == supported_streams.end()) {
                all_supported = false;
                break;
            }
        }

        // 如果所有流都支持，匹配成功
        if (all_supported) {
            serial = dev_serial;   // 将序列号返回给调用者
            return true;
        }
    }

    // 没有找到符合条件的设备
    return false;
}