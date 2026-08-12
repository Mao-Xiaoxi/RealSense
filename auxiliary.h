#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <set>

#include <librealsense2/rs.hpp>

class CLI {
public:
    CLI(const std::string& description) : description_(description) {}
    CLI& process(int argc, char** argv);
    std::string dump() const;
    // 查询某个选项的值，若不存在返回空字符串
    std::string get(const std::string& key) const {
        auto it = options_.find(key);
        return it != options_.end() ? it->second : "";
    }

private:
    std::string description_;
    std::string program_name_;
    std::vector<std::string> raw_args_;
    std::unordered_map<std::string, std::string> options_;

    void print_help() const;
};

bool device_with_streams(rs2::context& ctx, std::initializer_list<rs2_stream> streams, std::string& serial);
