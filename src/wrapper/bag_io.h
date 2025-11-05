//
// Created by xiang on 23-12-14.
//

#ifndef LIGHTNING_BAG_IO_H
#define LIGHTNING_BAG_IO_H

#include <functional>
#include <map>
#include <string>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/pointer_cast.hpp>
#include <deque>
#include <iostream>

#include <ros/serialization.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "msgs/custom_msg.hpp"
#include "msgs/serialization.hpp"
#include "ros/serialized_message_traits_patch.hpp"

#include "common/imu.h"
#include "common/odom.h"
#include "common/point_def.h"
#include "core/lightning_math.hpp"
#include "io/dataset_type.h"

namespace lightning {

/**
 * ROSBAG IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 * 现在可以指定ROS2
 */
class RosbagIO {
   public:
    explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::NCLT)
        : bag_file_(std::move(bag_file)) {
        /// handle ctrl-c
        // signal(SIGINT, lightning::debug::SigHandle);
    }

    using MsgType = boost::shared_ptr<ros::SerializedMessage const>;
    using MessageProcessFunction = std::function<bool(const MsgType &m)>;

    /// 一些方便直接使用的topics, messages
    // using Scan2DHandle = std::function<bool(sensor_msgs::msg::LaserScan::SharedPtr)>;

    // using PointCloud2Handle = std::function<bool(sensor_msgs::msg::PointCloud2::SharedPtr)>;
    using LivoxCloud2Handle = std::function<bool(msgs::CustomMsg::ConstPtr)>;
    // using FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
    using ImuHandle = std::function<bool(IMUPtr)>;
    using OdomHandle = std::function<bool(const OdomPtr &)>;

    /**
     * 遍历文件内容，调用回调函数
     * @param sleep_usec 每调用一个回调后的等待时间
     */
    void Go(int sleep_usec = 0);

    /// 通用处理函数
    RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) {
        process_func_.emplace(topic_name, func);
        return *this;
    }

    // /// point cloud 2 处理
    // RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) {
    //     return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
    //         auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    //         rclcpp::SerializedMessage data(*m->serialized_data);
    //         seri_cloud2_.deserialize_message(&data, msg.get());

    //         return f(msg);
    //     });
    // }

    /// livox 处理
    RosbagIO &AddLivoxCloudHandle(const std::string &topic_name, LivoxCloud2Handle f) {
        return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
            auto msg = boost::make_shared<msgs::CustomMsg>();

            ros::serialization::IStream stream(const_cast<uint8_t *>(m->buf.get()),
                                         static_cast<uint32_t>(m->num_bytes));
            ros::serialization::deserialize(stream, *msg);

            return f(boost::static_pointer_cast<const msgs::CustomMsg>(msg));
        });
    }

    RosbagIO &AddImuHandle(const std::string &topic_name, ImuHandle f) {
        return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
            auto msg = std::make_shared<msgs::Imu>();
            ros::serialization::IStream stream(const_cast<uint8_t *>(m->buf.get()),
                                         static_cast<uint32_t>(m->num_bytes));
            ros::serialization::deserialize(stream, *msg);

            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = msg->header.stamp.toSec();
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            return f(imu);
        });
    }

    /// odom 处理
    // RosbagIO &AddOdomHandle(const std::string &topic_name, OdomHandle f) {
    //     return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
    //         auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    //         rclcpp::SerializedMessage data(*m->serialized_data);
    //         seri_odom_.deserialize_message(&data, msg.get());

    //         /// nav_msg 的 odometry 转 odom
    //         return f(msg);
    //     });
    // }

    /// 清除现有的处理函数
    void CleanProcessFunc() { process_func_.clear(); }

   private:
    std::map<std::string, MessageProcessFunction> process_func_;

    /// 序列化
    // rclcpp::Serialization<nav_msgs::msg::Odometry> seri_odom_;
    // rclcpp::Serialization<sensor_msgs::msg::Imu> seri_imu_;
    // rclcpp::Serialization<sensor_msgs::msg::PointCloud2> seri_cloud2_;
    // rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> seri_livox_;

    std::string bag_file_;
    DatasetType dataset_type_ = DatasetType::NCLT;
};
}  // namespace lightning

#endif  // SLAM_ROS_BAG_IO_H
