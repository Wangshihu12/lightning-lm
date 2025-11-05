#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace msgs {

// ////////////// 基础头与时间 //////////////
struct Time {
    uint32_t sec{};
    uint32_t nsec{};

    double toSec() const {
        return static_cast<double>(sec) + static_cast<double>(nsec) / 1e9;
    }

    void fromSec(double t) {
        sec = static_cast<uint32_t>(t);  // 提取整数部分作为秒
        nsec = static_cast<uint32_t>((t - sec) * 1e9);  // 提取小数部分转换为纳秒
    }
};

struct Header {
    uint32_t seq{};
    Time stamp{};
    std::string frame_id;
};

// ////////////// Livox //////////////
struct CustomPoint {
    uint32_t offset_time; // ns or us，取决于驱动配置
    float x, y, z;
    uint8_t reflectivity;
    uint8_t tag;
    uint8_t line;
};

struct CustomMsg {
    Header header;
    uint64_t timebase;   // 驱动时间基准
    uint32_t point_num;  // 与 points.size() 一致
    uint8_t lidar_id;
    uint8_t rsvd[3];
    std::vector<CustomPoint> points;

    typedef boost::shared_ptr< ::msgs::CustomMsg > Ptr;
    typedef boost::shared_ptr< ::msgs::CustomMsg const > ConstPtr;
};

// ////////////// IMU //////////////
struct Quaterniond { double x{}, y{}, z{}, w{}; };
struct Vector3d { double x{}, y{}, z{}; };

struct Imu {
    Header header;
    Quaterniond orientation;
    std::array<double, 9> orientation_covariance{};
    Vector3d angular_velocity;
    std::array<double, 9> angular_velocity_covariance{};
    Vector3d linear_acceleration;
    std::array<double, 9> linear_acceleration_covariance{};

    typedef boost::shared_ptr< ::msgs::Imu > Ptr;
    typedef boost::shared_ptr< ::msgs::Imu const > ConstPtr;
};

// ////////////// Odometry //////////////
struct Odometry {
    Header header;
    std::string child_frame_id;
    Vector3d position;
    Quaterniond orientation;
    std::array<double, 9> covariance{};

    Vector3d linear;
    Vector3d angular;

    typedef boost::shared_ptr< ::msgs::Odometry > Ptr;
    typedef boost::shared_ptr< ::msgs::Odometry const > ConstPtr;
};

// ////////////// IterationStats //////////////
struct IterationStats {
    Header header;
    double translation_norm;
    double rotation_norm;
    double num_surf_from_scan;
    double num_corner_from_scan;
};

// ////////////// OptimizationStats //////////////
struct OptimizationStats {
    Header header;
    int laser_cloud_surf_from_map_num;
    int laser_cloud_corner_from_map_num;
    int laser_cloud_surf_stack_num;
    int laser_cloud_corner_stack_num;
    double total_translation;
    double total_rotation;
    double translation_from_last;
    double rotation_from_last;
    double time_elapsed;
    double latency;
    int n_iterations ;
    double average_distance ;
    double uncertainty_x;
    double uncertainty_y;
    double uncertainty_z;
    double uncertainty_roll;
    double uncertainty_pitch;
    double uncertainty_yaw;
    int plane_match_success;
    int plane_no_enough_neighbor;
    int plane_neighbor_too_far ;
    int plane_badpca_structure;
    int plane_invalid_numerical;
    int plane_mse_too_large;
    int plane_unknown;
    int prediction_source ;
    std::vector<IterationStats> iterations;
};

// ////////////// LaserFeature //////////////
struct LaserFeature {
    Header header;

    int sensor;

    int imu_available;
    int odom_available;

    // IMU initial guess for laser mapping
    double imu_quaternion_x;
    double imu_quaternion_y;
    double imu_quaternion_z;
    double imu_quaternion_w;

    // Odometry initial guess for laser mapping
    double initial_pose_x;
    double initial_pose_y;
    double initial_pose_z;
    double initial_quaternion_x;
    double initial_quaternion_y;
    double initial_quaternion_z;
    double initial_quaternion_w;

    // Preintegration reset ID
    int imu_preintegration_reset_id;

    // Point cloud messages
    CustomMsg cloud_nodistortion;  // original cloud remove distortion
    CustomMsg cloud_corner;    // extracted corner feature
    CustomMsg cloud_surface;   // extracted surface feature
    CustomMsg cloud_realsense;   // extracted surface feature from realsense

    typedef boost::shared_ptr< ::msgs::LaserFeature > Ptr;
    typedef boost::shared_ptr< ::msgs::LaserFeature const > ConstPtr;
};

} // namespace msgs
