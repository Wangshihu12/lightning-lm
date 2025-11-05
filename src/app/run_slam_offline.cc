//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#ifdef _WIN32
#define NOMINMAX  // 防止 Windows.h 定义 min/max 宏，避免与 C++ 标准库冲突
#include <windows.h>
#endif

#include "core/system/slam.h"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"

DEFINE_string(input_bag, "F:/mid360/m40_2025-09-08-07-41-42.goslam", "输入数据包路径");
DEFINE_string(config, "F:/github/lightning-lm/config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
#ifdef _WIN32
    // 设置控制台输出为 UTF-8 编码
    SetConsoleOutputCP(CP_UTF8);
    // 设置控制台输入为 UTF-8 编码（可选）
    SetConsoleCP(CP_UTF8);
#endif

    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::GLOG_INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据包";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    SlamSystem::Options options;
    options.online_mode_ = false;

    SlamSystem slam(options);

    /// 实时模式好像掉帧掉的比较厉害？

    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    slam.StartSLAM("new_map");

    lightning::YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");

    rosbag
        /// IMU 的处理
        .AddImuHandle(imu_topic,
                      [&slam](IMUPtr imu) {
                          slam.ProcessIMU(imu);
                          return true;
                      })

        /// livox 的处理
        .AddLivoxCloudHandle("/livox/lidar",
                             [&slam](msgs::CustomMsg::ConstPtr cloud) {
                                 slam.ProcessLidar(cloud);
                                 return true;
                             })
        .Go();

    slam.SaveMap("");
    Timer::PrintAll();

    LOG(INFO) << "done";

    return 0;
}
