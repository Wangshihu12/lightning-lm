//
// Created by xiang on 23-12-14.
//

#include "bag_io.h"

#include <glog/logging.h>
#include <filesystem>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

namespace lightning {

/**
 * [功能描述]：读取并处理ROS2 bag文件中的消息
 * @param sleep_usec：每读取一条消息后的休眠时间（微秒），用于控制消息处理速度，默认为0表示不休眠
 * @return 无返回值
 * 
 * 该函数会按顺序读取bag文件中的所有消息，对于每条消息：
 * 1. 根据topic名称查找对应的处理函数
 * 2. 如果找到处理函数则调用处理
 * 3. 根据设定的时间间隔进行休眠
 * 4. 检查退出标志，如果需要退出则提前返回
 */
void RosbagIO::Go(int sleep_usec) {
    // 将bag文件路径转换为filesystem路径对象
    std::filesystem::path p(bag_file_);
    
    // 创建ROS2 bag文件读取器，使用顺序读取模式
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    
    // 设置转换选项，使用CDR（Common Data Representation）序列化格式
    rosbag2_cpp::ConverterOptions cv_options{"cdr", "cdr"};
    
    // 打开bag文件，指定存储格式为sqlite3
    reader.open({bag_file_, "sqlite3"}, cv_options);

    // 循环读取bag文件中的所有消息
    while (reader.has_next()) {
        // 读取下一条消息
        auto msg = reader.read_next();
        
        // 在已注册的处理函数映射表中查找当前topic对应的处理函数
        auto iter = process_func_.find(msg->topic_name);
        
        // 如果找到对应的处理函数，则调用该函数处理消息
        if (iter != process_func_.end()) {
            iter->second(msg);
        }

        // 如果设置了休眠时间，则休眠指定的微秒数，用于控制处理速度
        if (sleep_usec > 0) {
            usleep(sleep_usec);
        }

        // 检查全局退出标志，如果需要退出则立即返回
        if (lightning::debug::flg_exit) {
            return;
        }
    }

    // 记录bag文件处理完成的日志信息
    LOG(INFO) << "bag " << bag_file_ << " finished.";
}

}  // namespace lightning