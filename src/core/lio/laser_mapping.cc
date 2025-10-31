#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "common/options.h"
#include "core/lightning_math.hpp"
#include "laser_mapping.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

namespace lightning {

/**
 * [功能描述]：初始化激光建图模块
 * @param config_yaml：YAML配置文件路径，包含LIO系统的所有参数配置
 * @return 成功返回true，失败返回false
 * 
 * 初始化流程：
 * 1. 从YAML文件加载参数配置
 * 2. 创建并初始化iVox增量体素地图结构（用于局部地图维护）
 * 3. 配置并初始化ESKF误差状态卡尔曼滤波器（用于状态估计）
 */
bool LaserMapping::Init(const std::string &config_yaml) {
    // 记录初始化日志，显示配置文件路径
    LOG(INFO) << "init laser mapping from " << config_yaml;
    
    // 从YAML配置文件加载参数，包括传感器参数、滤波器参数、地图参数等
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // 初始化局部地图结构（必须在LoadParams之后，因为需要使用加载的参数）
    // iVox是一种增量体素结构，用于高效的最近邻搜索和局部地图维护
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // 初始化ESKF（误差状态卡尔曼滤波器）配置选项
    ESKF::Options eskf_options;
    
    // 设置最大迭代次数，用于iterated ESKF的迭代优化
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    
    // 设置收敛阈值向量（23维状态的误差阈值）
    // epsi: 1e-3 * [1,1,...,1]^T，用于判断迭代是否收敛
    // 23维状态包括：位置(3) + 速度(3) + 旋转(3) + 加速度偏差(3) + 陀螺仪偏差(3) + 重力(3) + 外参平移(3) + 外参旋转(2)
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, 23, 1>::Ones();
    
    // 设置激光雷达观测模型函数（lambda表达式），用于计算残差和雅可比矩阵
    eskf_options.lidar_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) { ObsModel(s, obs); };
    
    // 设置是否使用Anderson加速（AA），可以加快ESKF迭代收敛速度
    eskf_options.use_aa_ = use_aa_;
    
    // 使用配置的选项初始化卡尔曼滤波器
    kf_.Init(eskf_options);

    // 初始化成功
    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_scan;
    Vec3d lidar_T_wrt_IMU;
    Mat3d lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        fasterlio::NUM_MAX_ITERATIONS = yaml["fasterlio"]["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = yaml["fasterlio"]["esti_plane_threshold"].as<float>();

        filter_size_scan = yaml["fasterlio"]["filter_size_scan"].as<float>();
        filter_size_map_min_ = yaml["fasterlio"]["filter_size_map"].as<float>();
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>();
        gyr_cov = yaml["fasterlio"]["gyr_cov"].as<float>();
        acc_cov = yaml["fasterlio"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["fasterlio"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["fasterlio"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
        lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();
        extrinsic_est_en_ = yaml["fasterlio"]["extrinsic_est_en"].as<bool>();
        extrinT_ = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["fasterlio"]["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["fasterlio"]["ivox_nearby_type"].as<int>();
        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();

        skip_lidar_num_ = yaml["fasterlio"]["skip_lidar_num"].as<int>();
        enable_skip_lidar_ = skip_lidar_num_ > 0;

    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    lidar_T_wrt_IMU = math::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = math::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));

    return true;
}

LaserMapping::LaserMapping(Options options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

/**
 * [功能描述]：处理IMU数据，进行状态预测和缓冲区管理
 * @param imu：IMU数据指针，包含时间戳、角速度和线性加速度
 * @return 无返回值
 * 
 * 处理流程：
 * 1. 检查时间戳合法性，处理时间回退情况
 * 2. 如果IMU已初始化，使用卡尔曼滤波器进行状态预测
 * 3. 更新可视化界面显示最新状态
 * 4. 将IMU数据加入缓冲区供后续处理使用
 */
void LaserMapping::ProcessIMU(const lightning::IMUPtr &imu) {
    // 增加发布计数器，用于统计处理的数据量
    publish_count_++;

    // 提取当前IMU数据的时间戳
    double timestamp = imu->timestamp;

    // 加锁保护共享的IMU缓冲区，防止多线程访问冲突
    UL lock(mtx_buffer_);
    
    // 检查时间戳是否发生回退（异常情况，可能由于数据重播或系统时间错误）
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        // 清空IMU缓冲区，避免时间戳顺序混乱
        imu_buffer_.clear();
    }

    // 检查IMU预处理模块是否已经完成初始化（需要一定数量的IMU数据来估计初始状态）
    if (p_imu_->IsIMUInited()) {
        /// 使用高频IMU卡尔曼滤波器预测最新的IMU状态
        // 根据时间增量、过程噪声协方差矩阵Q、角速度（rad/s）、线性加速度（m/s²）进行预测
        kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);

        // LOG(INFO) << "newest wrt lidar: " << timestamp - kf_.GetX().timestamp_;

        /// 如果启用了可视化界面，更新显示最新的导航状态（位姿、速度等）
        if (ui_) {
            ui_->UpdateNavState(kf_imu_.GetX());
        }
    }

    // 更新上一次处理的IMU时间戳，用于下次计算时间增量
    last_timestamp_imu_ = timestamp;

    // 将当前IMU数据添加到缓冲区尾部，供后续与激光雷达数据融合使用
    imu_buffer_.emplace_back(imu);
}

/**
 * [功能描述]：激光建图的主运行函数，执行LIO系统的核心处理流程
 * @return 成功返回true，失败或跳过返回false
 * 
 * 主要处理流程：
 * 1. 同步IMU和激光雷达数据
 * 2. IMU积分、卡尔曼滤波预测和点云运动畸变校正
 * 3. 处理第一帧扫描（初始化地图）
 * 4. 点云降采样
 * 5. IEKF迭代优化求解位姿
 * 6. 增量式更新局部地图
 * 7. 根据阈值判断是否生成新的关键帧
 * 8. 更新高频IMU状态估计
 */
bool LaserMapping::Run() {
    // 同步IMU和激光雷达数据包，确保数据时间对齐
    if (!SyncPackages()) {
        return false;
    }

    /// IMU数据处理：执行IMU积分、卡尔曼滤波预测、点云运动畸变校正
    p_imu_->Process(measures_, kf_, scan_undistort_);

    // 检查去畸变后的点云是否有效
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    /// 处理第一帧激光扫描（系统初始化）
    if (flg_first_scan_) {
        LOG(INFO) << "first scan pts: " << scan_undistort_->size();

        // 获取当前状态估计
        state_point_ = kf_.GetX();
        
        // 将第一帧点云从雷达坐标系转换到世界坐标系
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(scan_undistort_->points[i], scan_down_world_->points[i]);
        }
        
        // 将第一帧点云添加到iVox增量地图中，作为初始地图
        ivox_->AddPoints(scan_down_world_->points);

        // 记录第一帧激光雷达时间戳
        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        
        // 标记第一帧已处理完成
        flg_first_scan_ = false;
        return true;
    }

    // 如果启用了跳帧处理（用于降低计算负载）
    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % skip_lidar_num_;

        // 如果当前帧需要跳过（不是每skip_lidar_num_帧处理一次）
        if (skip_lidar_cnt_ != 0) {
            /// 更新UI中的内容，保持可视化实时性
            if (ui_) {
                ui_->UpdateNavState(kf_.GetX());
                ui_->UpdateScan(scan_undistort_, kf_.GetX().GetPose());
            }

            return false;
        }
    }

    // LOG(INFO) << "LIO get cloud at beg: " << std::setprecision(14) << measures_.lidar_begin_time_
    //           << ", end: " << measures_.lidar_end_time_;

    // 检测激光雷达数据断流（时间间隔超过0.5秒）
    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        LOG(ERROR) << "检测到雷达断流，时长：" << (measures_.lidar_begin_time_ - last_lidar_time_);
    }

    // 更新上一次激光雷达时间戳
    last_lidar_time_ = measures_.lidar_begin_time_;

    // 判断EKF是否已经完成初始化（需要运行足够长时间以收敛）
    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    /// 点云降采样（体素滤波），减少计算量
    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    // 获取降采样后的点数
    int cur_pts = scan_down_body_->size();
    
    // 如果点数太少（少于5个点），跳过本帧
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return false;
    }
    
    // 预分配内存，存储世界坐标系下的点云和最近邻点
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);

    // 执行IEKF（迭代扩展卡尔曼滤波）求解和状态更新
    Timer::Evaluate(
        [&, this]() {
            // 预分配成员变量内存，提高效率
            residuals_.resize(cur_pts, 0);                          // 每个点的残差
            point_selected_surf_.resize(cur_pts, true);             // 每个点是否被选为有效特征点
            plane_coef_.resize(cur_pts, Vec4f::Zero());             // 每个点对应的平面系数 [nx, ny, nz, d]

            // 保存更新前的状态，用于后续判断是否需要回退
            auto old_state = kf_.GetX();

            // 执行IEKF更新：使用激光雷达观测数据进行迭代优化
            // 1e-3是收敛阈值，用于判断迭代是否收敛
            kf_.Update(ESKF::ObsType::LIDAR, 1e-3);
            state_point_ = kf_.GetX();

            // 如果启用了保持初始IMU估计，并且关键帧数量较少，检查旋转变化是否过大
            // 如果旋转变化超过0.3度，认为更新不可靠，回退到预测状态
            if (keep_first_imu_estimation_ && all_keyframes_.size() < 5 &&
                (old_state.rot_.inverse() * state_point_.rot_).log().norm() > 0.3 * M_PI / 180) {
                kf_.ChangeX(old_state);
                state_point_ = old_state;

                LOG(INFO) << "set state as prediction";
            }

            // LOG(INFO) << "old yaw: " << old_state.rot_.angleZ() << ", new: " << state_point_.rot_.angleZ();

            // 设置状态时间戳为激光雷达结束时间
            state_point_.timestamp_ = measures_.lidar_end_time_;
            
            // 更新当前欧拉角（旋转）
            euler_cur_ = state_point_.rot_;
            
            // 计算激光雷达在世界坐标系下的位置：机体位置 + 旋转 * 外参平移
            pos_lidar_ = state_point_.pos_ + state_point_.rot_ * state_point_.offset_t_lidar_;
        },
        "IEKF Solve and Update");

    // 更新局部地图：将当前帧点云增量式地加入到iVox地图中
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    // 输出建图统计信息：输入点数、降采样后点数、地图网格数、有效特征点数
    LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;

    /// 关键帧生成判断
    if (last_kf_ == nullptr) {
        // 如果还没有关键帧，创建第一个关键帧
        MakeKF();
    } else {
        // 获取上一个关键帧和当前帧的位姿
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = state_point_.GetPose();
        
        // 判断是否满足生成新关键帧的条件：
        // 1. 平移距离超过阈值 kf_dis_th_
        // 2. 旋转角度超过阈值 kf_angle_th_
        if ((last_pose.translation() - cur_pose.translation()).norm() > options_.kf_dis_th_ ||
            (last_pose.so3().inverse() * cur_pose.so3()).log().norm() > options_.kf_angle_th_) {
            MakeKF();
        } 
        // 在定位模式下，如果时间间隔超过2秒也生成关键帧
        else if (!options_.is_in_slam_mode_ && (state_point_.timestamp_ - last_kf_->GetState().timestamp_) > 2.0) {
            MakeKF();
        }
    }

    /// 更新高频IMU状态估计器（kf_imu_），用于IMU数据到来时的高频状态预测
    kf_imu_ = kf_;
    
    // 如果有新的IMU数据，将缓冲区中的IMU数据向前传播
    if (!measures_.imu_.empty()) {
        // 从当前测量包中最后一个IMU时间戳开始
        double t = measures_.imu_.back()->timestamp;
        
        // 遍历IMU缓冲区中的所有数据，逐步预测状态
        for (auto &imu : imu_buffer_) {
            double dt = imu->timestamp - t;
            // 使用IMU数据进行卡尔曼滤波预测
            kf_imu_.Predict(dt, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);
            t = imu->timestamp;
        }
    }

    // 如果启用了可视化，更新显示当前扫描结果
    if (ui_) {
        ui_->UpdateScan(scan_undistort_, state_point_.GetPose());
    }

    // 处理成功
    return true;
}

/**
 * [功能描述]：创建新的关键帧并更新位姿
 * @return 无返回值
 * 
 * 主要功能：
 * 1. 创建包含当前点云和状态的关键帧
 * 2. 计算并设置优化位姿（基于上一关键帧的位姿递推）
 * 3. 在SLAM模式下将关键帧加入全局列表
 * 4. 更新最后一个关键帧的引用
 */
void LaserMapping::MakeKF() {
    // 创建新的关键帧对象，包含关键帧ID、去畸变点云和当前状态估计
    Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, scan_undistort_, state_point_);

    // 如果存在上一个关键帧，计算位姿增量并传播优化位姿
    if (last_kf_) {
        // LOG(INFO) << "last kf lio: " << last_kf_->GetLIOPose().translation().transpose()
        //           << ", opt: " << last_kf_->GetOptPose().translation().transpose();

        /// 计算相对位姿变换（从上一关键帧的LIO位姿到当前关键帧的LIO位姿的增量）
        SE3 delta = last_kf_->GetLIOPose().inverse() * kf->GetLIOPose();
        
        // 将位姿增量应用到上一关键帧的优化位姿上，得到当前关键帧的优化位姿初值
        // 优化位姿会在后续的回环检测和位姿图优化中被进一步调整
        kf->SetOptPose(last_kf_->GetOptPose() * delta);
    } else {
        // 如果是第一个关键帧，将优化位姿直接设置为LIO位姿
        kf->SetOptPose(kf->GetLIOPose());
    }

    // 设置关键帧的完整状态信息（包括位置、速度、旋转、IMU偏差等）
    kf->SetState(state_point_);

    // 记录关键帧创建日志，输出ID、状态位置、优化位姿、LIO位姿等信息
    LOG(INFO) << "LIO: create kf " << kf->GetID() << ", state: " << state_point_.pos_.transpose()
              << ", kf opt pose: " << kf->GetOptPose().translation().transpose()
              << ", lio pose: " << kf->GetLIOPose().translation().transpose();

    // 如果处于SLAM建图模式，将关键帧加入全局关键帧列表
    // （定位模式下不需要保存所有关键帧）
    if (options_.is_in_slam_mode_) {
        all_keyframes_.emplace_back(kf);
    }

    // 更新最后一个关键帧的引用，供下次创建关键帧时使用
    last_kf_ = kf;
}

void LaserMapping::ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
                      << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            // LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
            //           << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(CloudPtr cloud) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;

            double timestamp = math::ToSec(cloud->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / double(1000);
            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        lo::lidar_time_interval = lidar_mean_scantime_;

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) {
            break;
        }

        measures_.imu_.push_back(imu_buffer_.front());

        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    // LOG(INFO) << "sync: " << std::setprecision(14) << measures_.lidar_begin_time_ << ", " <<
    // measures_.lidar_end_time_;

    return true;
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    size_t cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(scan_down_body_->points[i], scan_down_world_->points[i]);

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = math::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= fasterlio::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < fasterlio::NUM_MATCH_POINTS; readd_i++) {
                    if (math::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }

            if (need_add) {
                points_to_add.emplace_back(point_world);  // FIXME 这并发可能有点问题
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * [功能描述]：激光雷达点云配准观测模型
 * @param s：导航状态，包含位置、旋转、IMU偏差、外参等
 * @param obs：自定义观测模型数据结构，用于存储雅可比矩阵H和残差
 * @return 无返回值
 * 
 * 该函数被ESKF的自定义观测模型调用，执行以下操作：
 * 1. 将点云从雷达坐标系转换到世界坐标系
 * 2. 在局部地图中查找最近邻点并拟合平面
 * 3. 计算点到平面的残差（距离）
 * 4. 计算观测模型的雅可比矩阵H
 * 
 * 这是IEKF中的观测更新步骤，通过点云配准约束来修正状态估计
 */
void LaserMapping::ObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    // 获取降采样后的点云数量
    int cnt_pts = scan_down_body_->size();

    // 创建索引数组，用于并行处理
    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    // 第一阶段：激光雷达点云匹配，计算点到平面的残差
    Timer::Evaluate(
        [&, this]() {
            // 计算世界坐标系到雷达坐标系的旋转矩阵：R_wl = R_wb * R_bl
            auto R_wl = (s.rot_ * s.offset_R_lidar_).cast<float>();
            
            // 计算雷达在世界坐标系下的位置：t_wl = R_wb * t_bl + t_wb
            auto t_wl = (s.rot_ * s.offset_t_lidar_ + s.pos_).cast<float>();

            // 并行处理每个点（使用无序并行执行策略提高效率）
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];      // 雷达坐标系下的点
                PointType &point_world = scan_down_world_->points[i];    // 世界坐标系下的点

                /* 将点从雷达坐标系转换到世界坐标系 */
                Vec3f p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;     // p_w = R_wl * p_b + t_wl
                point_world.intensity = point_body.intensity;             // 保留强度信息

                // 获取当前点的最近邻点容器
                auto &points_near = nearest_points_[i];

                /** 在局部地图中查找最近的表面点 **/
                // 从iVox地图中获取最近邻点（数量为NUM_MATCH_POINTS）
                ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
                
                // 检查最近邻点数量是否足够进行平面拟合
                point_selected_surf_[i] = points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;
                
                // 如果最近邻点足够，尝试拟合平面
                if (point_selected_surf_[i]) {
                    // 使用最小二乘法拟合平面，得到平面系数 [nx, ny, nz, d]
                    // 平面方程：nx*x + ny*y + nz*z + d = 0
                    point_selected_surf_[i] =
                        math::esti_plane(plane_coef_[i], points_near, fasterlio::ESTI_PLANE_THRESHOLD);
                }

                // 如果平面拟合成功，计算点到平面的距离（残差）
                if (point_selected_surf_[i]) {
                    // 将点坐标转换为齐次坐标 [x, y, z, 1]
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    
                    // 计算点到平面的有符号距离：d = n^T * p + d
                    float pd2 = plane_coef_[i].dot(temp);

                    // 验证对应关系的有效性：距离应该与点的深度成合理比例
                    // 这个判据用于过滤不可靠的匹配（81是经验系数）
                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;  // 保存残差
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    // 统计有效特征点数量
    effect_feat_num_ = 0;

    // 预分配内存，存储对应点和平面法向量
    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    
    // 筛选有效的特征点，压缩数组
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            // 保存平面法向量（含d值）：[nx, ny, nz, d]
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            
            // 保存雷达坐标系下的点坐标：[x, y, z, residual]
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];  // 将残差存储在第4个分量

            effect_feat_num_++;
        }
    }
    
    // 调整数组大小，移除无效点
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    // 如果没有有效特征点，观测无效，返回
    if (effect_feat_num_ < 1) {
        obs.valid_ = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    // 第二阶段：计算雅可比矩阵H和残差向量
    Timer::Evaluate(
        [&, this]() {
            /*** 计算观测模型的雅可比矩阵H和测量向量 ***/
            // 雅可比矩阵维度：effect_feat_num_ × 12
            // 12维包括：位置(3) + 旋转(3) + 外参旋转(3) + 外参平移(3)
            obs.h_x_ = Eigen::MatrixXd::Zero(effect_feat_num_, 12);
            obs.residual_.resize(effect_feat_num_);

            // 调整索引数组大小为有效特征点数量
            index.resize(effect_feat_num_);
            
            // 预计算常用变换矩阵，避免重复计算
            const Mat3f off_R = s.offset_R_lidar_.matrix().cast<float>();    // 雷达外参旋转矩阵
            const Vec3f off_t = s.offset_t_lidar_.cast<float>();             // 雷达外参平移向量
            const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();      // 世界到机体的旋转矩阵

            // 并行计算每个有效特征点的雅可比矩阵
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                // 获取雷达坐标系下的点坐标
                Vec3f point_this_be = corr_pts_[i].head<3>();
                
                // 计算点的反对称矩阵（用于叉乘运算）
                Mat3f point_be_crossmat = math::SKEW_SYM_MATRIX(point_this_be);
                
                // 将点从雷达坐标系转换到机体坐标系
                Vec3f point_this = off_R * point_this_be + off_t;
                Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);

                /*** 获取最近平面的法向量 ***/
                Vec3f norm_vec = corr_norm_[i].head<3>();  // [nx, ny, nz]

                /*** 计算观测模型的雅可比矩阵H ***/
                // C = R_wb^T * n：法向量在机体坐标系下的表示
                Vec3f C(Rt * norm_vec);
                
                // A = [p]_× * C：对旋转的雅可比
                Vec3f A(point_crossmat * C);

                // 根据是否估计外参，填充雅可比矩阵的不同列
                if (extrinsic_est_en_) {
                    // 如果估计外参，计算外参相关的雅可比
                    // B = [p_b]_× * R_bl^T * C：对外参旋转的雅可比
                    Vec3f B(point_be_crossmat * off_R.transpose() * C);
                    
                    // 雅可比矩阵H的第i行：[∂r/∂p, ∂r/∂θ, ∂r/∂θ_bl, ∂r/∂t_bl]
                    obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0], B[1],
                        B[2], C[0], C[1], C[2];
                } else {
                    // 如果不估计外参，外参相关的雅可比置零
                    obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0;
                }

                /*** 测量值：点到最近平面/角点的距离（残差）***/
                obs.residual_(i) = -corr_pts_[i][3];  // 取负号是因为残差定义为 r = -(n^T*p + d)
            });
        },
        "    ObsModel (IEKF Build Jacobian)");

    /// 计算并填入残差统计信息（用于自适应调整观测噪声）
    std::vector<double> res_sq2;
    for (size_t i = 0; i < cnt_pts; ++i) {
        if (point_selected_surf_[i]) {
            double r = residuals_[i];
            res_sq2.emplace_back(r * r);  // 保存残差的平方
        }
    }

    // 对残差平方排序，用于计算中位数和最大值
    std::sort(res_sq2.begin(), res_sq2.end());
    
    // 保存中位数残差平方（鲁棒的误差估计）
    obs.lidar_residual_mean_ = res_sq2[res_sq2.size() / 2];
    
    // 保存最大残差平方（用于检测异常）
    obs.lidar_residual_max_ = res_sq2[res_sq2.size() - 1];
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////

CloudPtr LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res) {
    CloudPtr global_map(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);

    for (auto &kf : all_keyframes_) {
        CloudPtr cloud = kf->GetCloud();

        CloudPtr cloud_filter(new PointCloudType);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);

        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        if (use_lio_pose) {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetLIOPose().matrix());
        } else {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetOptPose().matrix());
        }

        *global_map += *cloud_trans;
    }

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    LOG(INFO) << "global map: " << global_map_filtered->size();

    return global_map_filtered;
}

void LaserMapping::SaveMap() {
    /// 保存地图
    auto global_map = GetGlobalMap(true);

    pcl::io::savePCDFileBinaryCompressed("./data/lio.pcd", *global_map);

    LOG(INFO) << "lio map is saved to ./data/lio.pcd";
}

CloudPtr LaserMapping::GetRecentCloud() {
    if (lidar_buffer_.empty()) {
        return nullptr;
    }

    return lidar_buffer_.front();
}

}  // namespace lightning