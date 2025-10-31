//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

/**
 * [功能描述]：初始化SLAM系统的所有模块和组件
 * @param yaml_path：配置文件的路径，包含系统各模块的参数配置
 * @return 成功返回true，失败返回false
 * 
 * 初始化流程包括：
 * 1. 初始化激光惯性里程计（LIO）模块
 * 2. 加载配置文件，读取系统选项
 * 3. 根据配置初始化可选模块：回环检测、3D可视化、2D网格地图、在线ROS2节点等
 */
bool SlamSystem::Init(const std::string& yaml_path) {
    // 创建激光惯性里程计（Laser-Inertial Odometry）模块实例
    lio_ = std::make_shared<LaserMapping>();
    
    // 初始化LIO模块，若初始化失败则返回错误
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    // 加载YAML配置文件
    auto yaml = YAML::LoadFile(yaml_path);
    
    // 从配置文件读取系统选项参数
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();      // 是否启用回环检测
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();               // 是否启用3D可视化界面
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();           // 是否启用2D可视化界面
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();                   // 是否启用G2P5网格地图
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();                    // 是否在关键帧处暂停等待用户操作

    // 根据配置初始化回环检测模块
    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        
        // 创建回环检测选项并设置在线模式标志
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        
        // 创建并初始化回环检测模块
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    // 根据配置初始化3D可视化界面
    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        
        // 创建Pangolin 3D可视化窗口
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        // 将可视化界面设置到LIO模块，使其能够实时显示建图结果
        lio_->SetUI(ui_);
    }

    // 根据配置初始化G2P5网格地图模块
    if (options_.with_gridmap_) {
        // 创建G2P5配置选项
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        // 创建并初始化G2P5网格地图模块
        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        // 如果同时启用了回环检测，设置回环闭合回调函数
        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次重绘，更新全局地图
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        // 如果启用了2D可视化，设置地图更新回调函数
        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                // 将网格地图转换为OpenCV图像格式
                cv::Mat image = map->ToCV();
                
                // 显示2D地图
                cv::imshow("map", image);

                // 根据配置决定是否在每个关键帧处暂停
                if (options_.step_on_kf_) {
                    cv::waitKey(0);      // 等待用户按键继续（用于调试）
                } else {
                    cv::waitKey(10);     // 等待10ms刷新显示
                }
            });
        }
    }

    // 如果是在线模式，创建ROS2节点和订阅器
    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// 创建ROS2节点，节点名为"lightning_slam"
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");

        // 从配置文件读取传感器话题名称
        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();                    // IMU话题
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();                // 激光雷达点云话题
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();          // Livox雷达话题

        // 设置ROS2 QoS（服务质量）策略，队列大小为10
        rclcpp::QoS qos(10);
        // qos.best_effort();  // 可选：设置为尽力而为模式

        // 创建IMU数据订阅器
        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                // 创建IMU数据结构
                IMUPtr imu = std::make_shared<IMU>();
                
                // 转换时间戳为秒
                imu->timestamp = ToSec(msg->header.stamp);
                
                // 提取线性加速度（m/s²），转换为Vec3d格式
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                
                // 提取角速度（rad/s），转换为Vec3d格式
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                // 处理IMU数据
                ProcessIMU(imu);
            });

        // 创建标准点云订阅器（sensor_msgs::PointCloud2格式）
        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                // 使用计时器评估点云处理性能，记录处理时间
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        // 创建Livox自定义点云订阅器（Livox专用格式）
        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                // 使用计时器评估点云处理性能
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        // 创建保存地图服务，响应外部保存地图请求
        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        LOG(INFO) << "online slam node has been created.";
    }

    // 初始化成功
    return true;
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

/**
 * [功能描述]：处理Livox激光雷达的自定义格式点云数据
 * @param cloud：Livox雷达的自定义消息格式点云数据（SharedPtr智能指针）
 * @return 无返回值
 * 
 * 处理流程：
 * 1. 检查系统运行状态
 * 2. 将点云数据输入LIO模块进行处理
 * 3. 运行LIO算法，进行位姿估计和建图
 * 4. 获取生成的关键帧并分发到各个模块（回环检测、网格地图、可视化）
 */
void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    // 检查SLAM系统是否处于运行状态，如果未运行则直接返回
    if (running_ == false) {
        return;
    }

    // 将Livox点云数据传递给LIO模块进行预处理
    lio_->ProcessPointCloud2(cloud);
    
    // 运行LIO算法，执行IMU积分、点云配准、位姿优化等操作
    lio_->Run();

    // 获取LIO模块生成的最新关键帧
    auto kf = lio_->GetKeyframe();
    
    // 检查是否生成了新的关键帧
    if (kf != cur_kf_) {
        // 更新当前关键帧指针
        cur_kf_ = kf;
    } else {
        // 如果没有新的关键帧生成，直接返回
        return;
    }

    // 验证关键帧指针的有效性
    if (cur_kf_ == nullptr) {
        return;
    }

    // 如果启用了回环检测模块，将新关键帧加入回环检测队列
    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    // 如果启用了网格地图模块，将新关键帧推送到G2P5模块进行地图更新
    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    // 如果启用了3D可视化界面，更新显示的关键帧信息
    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
