//
// Created by xiang on 23-12-14.
//

#include "bag_io.h"

#include <glog/logging.h>
#include <filesystem>
#include <chrono>

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
    
    rosbag::Bag bag;
    bag.open(bag_file_, rosbag::bagmode::Read);

    std::vector<std::string> topics{"/livox/lidar", "/livox/imu"};

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // 循环读取bag文件中的所有消息
    BOOST_FOREACH (rosbag::MessageInstance const m, view) {
        boost::shared_ptr<ros::SerializedMessage const> msg =
          m.instantiate<ros::SerializedMessage>();

        const std::string &datatype = m.getDataType();
        const std::string &topic = m.getTopic();
        
        // 在已注册的处理函数映射表中查找当前topic对应的处理函数
        auto iter = process_func_.find(topic);
        
        // 如果找到对应的处理函数，则调用该函数处理消息
        if (iter != process_func_.end()) {
            iter->second(msg);
        }

        // 如果设置了休眠时间，则休眠指定的微秒数，用于控制处理速度
        if (sleep_usec > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_usec));
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