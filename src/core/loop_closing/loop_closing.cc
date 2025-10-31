//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/pointcloud_utils.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>

#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/cauchy.h"
#include "core/types/edge_se3.h"
#include "core/types/vertex_se3.h"
#include "io/yaml_io.h"

namespace lightning {

LoopClosing::~LoopClosing() {
    if (options_.online_mode_) {
        kf_thread_.Quit();
    }
}

/**
 * [功能描述]：初始化回环检测模块
 * @param yaml_path：YAML配置文件路径，包含回环检测的各项参数
 * @return 无返回值
 * 
 * 初始化流程：
 * 1. 配置位姿图优化器（使用Levenberg-Marquardt算法）
 * 2. 设置运动约束和回环约束的信息矩阵
 * 3. 从YAML文件加载回环检测参数
 * 4. 在在线模式下启动异步处理线程
 */
void LoopClosing::Init(const std::string yaml_path) {
    /// 配置并创建Miao优化器（用于位姿图优化）
    // 使用Levenberg-Marquardt非线性优化算法和稀疏Eigen线性求解器
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    
    // 启用增量模式，允许动态添加节点和边
    config.incremental_mode_ = true;
    
    // 创建优化器：6维表示SE3位姿（3维平移+3维旋转），3维表示点特征
    optimizer_ = miao::SetupOptimizer<6, 3>(config);

    // 初始化运动约束的信息矩阵（6x6）
    // 信息矩阵 = 协方差矩阵的逆，用于表示约束的置信度
    info_motion_.setIdentity();
    
    // 设置平移部分的信息矩阵（前3x3块），权重与平移噪声成反比
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise_ * options_.motion_trans_noise_);
    
    // 设置旋转部分的信息矩阵（后3x3块），权重与旋转噪声成反比
    info_motion_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.motion_rot_noise_ * options_.motion_rot_noise_);

    // 初始化回环约束的信息矩阵（6x6）
    info_loops_.setIdentity();
    
    // 设置回环平移约束的信息矩阵（前3x3块）
    info_loops_.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (options_.loop_trans_noise_ * options_.loop_trans_noise_);
    
    // 设置回环旋转约束的信息矩阵（后3x3块）
    info_loops_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.loop_rot_noise_ * options_.loop_rot_noise_);

    // 如果提供了配置文件路径，从YAML文件加载回环检测参数
    if (!yaml_path.empty()) {
        YAML_IO yaml(yaml_path);

        options_.loop_kf_gap_ = yaml.GetValue<int>("loop_closing", "loop_kf_gap");           // 检测回环的关键帧间隔
        options_.min_id_interval_ = yaml.GetValue<int>("loop_closing", "min_id_interval");   // 回环候选的最小ID间隔
        options_.closest_id_th_ = yaml.GetValue<int>("loop_closing", "closest_id_th");       // 最近邻ID阈值
        options_.max_range_ = yaml.GetValue<double>("loop_closing", "max_range");            // 回环检测的最大距离范围（米）
        options_.ndt_score_th_ = yaml.GetValue<double>("loop_closing", "ndt_score_th");      // NDT配准得分阈值
    }

    // 如果是在线模式，启动异步处理线程
    if (options_.online_mode_) {
        LOG(INFO) << "loop closing module is running in online mode";
        
        // 设置关键帧处理函数
        kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { HandleKF(kf); });
        
        // 设置线程名称，便于调试
        kf_thread_.SetName("handle loop closure");
        
        // 启动线程，开始异步处理关键帧
        kf_thread_.Start();
    }
}

void LoopClosing::AddKF(Keyframe::Ptr kf) {
    if (options_.online_mode_) {
        kf_thread_.AddMessage(kf);
    } else {
        HandleKF(kf);
    }
}

void LoopClosing::HandleKF(Keyframe::Ptr kf) {
    if (kf == last_kf_) {
        return;
    }

    cur_kf_ = kf;
    all_keyframes_.emplace_back(kf);

    // 检测回环候选
    DetectLoopCandidates();

    if (options_.verbose_) {
        LOG(INFO) << "lc: get kf " << cur_kf_->GetID() << " candi: " << candidates_.size();
    }

    // 计算回环位姿
    ComputeLoopCandidates();

    // 位姿图优化
    PoseOptimization();

    last_kf_ = kf;
}

/**
 * [功能描述]：检测回环候选关键帧
 * @return 无返回值
 * 
 * 检测流程：
 * 1. 清空候选列表
 * 2. 检查与上次回环的关键帧间隔是否足够
 * 3. 遍历所有历史关键帧，根据多个条件筛选回环候选：
 *    - 同一轨迹内的ID间隔要求
 *    - 与当前帧的ID差值阈值
 *    - 空间距离约束（基于x-y平面距离）
 * 4. 为满足条件的关键帧创建回环候选，计算相对位姿
 * 5. 更新上次回环关键帧记录
 */
void LoopClosing::DetectLoopCandidates() {
    // 清空回环候选列表，准备检测新的回环
    candidates_.clear();

    // 获取所有关键帧的引用
    auto& kfs_mapping = all_keyframes_;
    
    // 记录第一个满足条件的关键帧，用于后续同轨迹内的间隔判断
    Keyframe::Ptr check_first = nullptr;

    // 如果还没有上次回环关键帧，将当前关键帧设为上次回环关键帧并返回
    // 这是初始化状态，避免第一帧就进行回环检测
    if (last_loop_kf_ == nullptr) {
        last_loop_kf_ = cur_kf_;
        return;
    }

    // 检查当前关键帧与上次回环关键帧的ID间隔是否足够
    // 如果间隔太小，跳过本次回环检测，避免过于频繁的回环检测
    if (last_loop_kf_ && (cur_kf_->GetID() - last_loop_kf_->GetID()) <= options_.loop_kf_gap_) {
        LOG(INFO) << "skip because last loop kf: " << last_loop_kf_->GetID();
        return;
    }

    // 遍历所有历史关键帧，寻找回环候选
    for (auto kf : kfs_mapping) {
        // 如果已经找到一个候选，检查当前关键帧与第一个候选的ID间隔
        // 在同一条轨迹内，跳过ID间隔太小的关键帧，避免冗余的回环检测
        if (check_first != nullptr && abs(int(kf->GetID() - check_first->GetID())) <= options_.min_id_interval_) {
            continue;
        }

        // 检查历史关键帧与当前关键帧的ID差值
        // 如果ID差值小于阈值，说明在同一条轨迹中且时间太近，不考虑回环
        // 使用break而不是continue，因为关键帧是按ID顺序排列的
        if (abs(int(kf->GetID() - cur_kf_->GetID())) < options_.closest_id_th_) {
            break;
        }

        // 计算历史关键帧与当前关键帧之间的位移向量
        Vec3d dt = kf->GetOptPose().translation() - cur_kf_->GetOptPose().translation();
        
        // 计算x-y平面上的距离（2D距离），忽略z方向的高度差
        double t2d = dt.head<2>().norm();
        
        // 获取最大检测范围阈值
        double range_th = options_.max_range_;

        // 如果2D距离小于阈值，说明空间位置足够接近，可以作为回环候选
        if (t2d < range_th) {
            // 创建回环候选对象，记录历史关键帧ID和当前关键帧ID
            LoopCandidate c(kf->GetID(), cur_kf_->GetID());
            
            // 计算相对位姿变换（从历史关键帧到当前关键帧）
            // 使用LIO位姿进行初始估计
            c.Tij_ = kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();

            // 将候选添加到候选列表
            candidates_.emplace_back(c);
            
            // 记录第一个找到的候选关键帧
            check_first = kf;
        }
    }

    // 如果找到了回环候选，更新上次回环关键帧为当前关键帧
    if (!candidates_.empty()) {
        last_loop_kf_ = cur_kf_;
    }

    // 如果启用了详细日志且找到了候选，输出候选数量
    if (options_.verbose_ && !candidates_.empty()) {
        LOG(INFO) << "lc candi: " << candidates_.size();
    }
}

/**
 * [功能描述]：计算并验证回环候选
 * @return 无返回值
 * 
 * 处理流程：
 * 1. 检查是否存在候选
 * 2. 对每个候选执行NDT配准计算相对位姿
 * 3. 根据NDT配准得分筛选成功的候选
 * 4. 更新候选列表，仅保留通过验证的回环
 */
void LoopClosing::ComputeLoopCandidates() {
    // 如果没有回环候选，直接返回
    if (candidates_.empty()) {
        return;
    }

    // 对每个候选执行计算：使用NDT算法进行点云配准，计算精确的相对位姿和配准得分
    std::for_each(candidates_.begin(), candidates_.end(), [this](LoopCandidate& c) { ComputeForCandidate(c); });
    
    // 创建临时容器保存配准成功的候选
    std::vector<LoopCandidate> succ_candidates;
    
    // 遍历所有候选，根据NDT配准得分进行筛选
    for (const auto& lc : candidates_) {
        // 输出候选的关键帧ID对和NDT配准得分
        LOG(INFO) << "candi " << lc.idx1_ << ", " << lc.idx2_ << " s: " << lc.ndt_score_;
        
        // 如果NDT配准得分高于阈值，认为回环有效
        // NDT得分越高，表示点云配准质量越好，回环越可靠
        if (lc.ndt_score_ > options_.ndt_score_th_) {
            succ_candidates.emplace_back(lc);
        }
    }

    // 如果启用了详细日志，输出成功率统计
    if (options_.verbose_) {
        LOG(INFO) << "success: " << succ_candidates.size() << "/" << candidates_.size();
    }

    // 将成功的候选交换到原候选列表，丢弃配准失败的候选
    candidates_.swap(succ_candidates);
}

/**
 * [功能描述]：对单个回环候选进行NDT配准计算
 * @param c：回环候选对象，包含候选关键帧ID对，函数会更新其相对位姿和配准得分
 * @return 无返回值
 * 
 * 处理流程：
 * 1. 构建子地图：为历史关键帧构建局部子地图，融合周围关键帧的点云
 * 2. 多分辨率NDT配准：从粗到精逐步配准，提高精度和鲁棒性
 * 3. 计算相对位姿：将配准结果转换为关键帧间的相对位姿约束
 */
void LoopClosing::ComputeForCandidate(lightning::LoopCandidate& c) {
    // 输出配准日志，显示正在对齐的关键帧ID对
    LOG(INFO) << "aligning " << c.idx1_ << " with " << c.idx2_;
    
    // 子地图范围：在目标关键帧周围±40帧范围内构建子地图
    const int submap_idx_range = 40;
    
    // 获取回环候选的两个关键帧
    auto kf1 = all_keyframes_.at(c.idx1_), kf2 = all_keyframes_.at(c.idx2_);

    /**
     * [Lambda函数]：构建子地图
     * @param given_id：中心关键帧ID
     * @param build_in_world：是否在世界坐标系下构建（true）或在局部坐标系下构建（false）
     * @return 构建的子地图点云
     * 
     * 子地图通过融合中心关键帧周围多个关键帧的点云构建，提供更丰富的几何信息用于配准
     */
    auto build_submap = [this](int given_id, bool build_in_world) -> CloudPtr {
        // 创建空的子地图点云
        CloudPtr submap(new PointCloudType);
        
        // 遍历中心关键帧周围的关键帧（每隔4帧采样一次，降低计算量）
        for (int idx = -submap_idx_range; idx < submap_idx_range; idx += 4) {
            int id = idx + given_id;
            
            // 检查ID是否越界
            if (id < 0 || id > all_keyframes_.size()) {
                continue;
            }

            // 获取当前关键帧及其点云
            auto kf = all_keyframes_[id];
            CloudPtr cloud = kf->GetCloud();

            // RemoveGround(cloud, 0.1);  // 可选：移除地面点

            // 跳过空点云
            if (cloud->empty()) {
                continue;
            }

            // 获取当前关键帧的LIO位姿（世界坐标系下）
            SE3 Twb = kf->GetLIOPose();

            // 如果不在世界坐标系下构建，则转换到局部坐标系
            // 局部坐标系以given_id关键帧为原点
            if (!build_in_world) {
                Twb = all_keyframes_.at(given_id)->GetLIOPose().inverse() * Twb;
            }

            // 将点云从雷达坐标系转换到目标坐标系
            CloudPtr cloud_trans(new PointCloudType);
            pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());

            // 将转换后的点云累加到子地图中
            *submap += *cloud_trans;
        }
        return submap;
    };

    // 为历史关键帧kf1构建世界坐标系下的子地图（目标点云）
    auto submap_kf1 = build_submap(kf1->GetID(), true);

    // 获取当前关键帧kf2的点云（源点云）
    CloudPtr submap_kf2 = kf2->GetCloud();

    // 检查点云有效性，如果任一点云为空则配准失败
    if (submap_kf1->empty() || submap_kf2->empty()) {
        c.ndt_score_ = 0;
        return;
    }

    // 获取kf2的LIO位姿作为NDT配准的初始变换矩阵（4x4）
    Mat4f Tw2 = kf2->GetLIOPose().matrix().cast<float>();

    /// 多分辨率NDT配准：从粗到精逐步优化
    // 输出点云（配准后的源点云）
    CloudPtr output(new PointCloudType);
    
    // 分辨率序列：10m -> 5m -> 2m -> 1m，逐步细化
    std::vector<double> res{10.0, 5.0, 2.0, 1.0};

    // 降采样后的粗糙地图
    CloudPtr rough_map1, rough_map2;

    // 遍历每个分辨率级别进行配准
    for (auto& r : res) {
        // 创建NDT（正态分布变换）配准对象
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        
        // 设置收敛判断阈值：变换矩阵变化小于0.05时认为收敛
        ndt.setTransformationEpsilon(0.05);
        
        // 设置牛顿法优化的步长
        ndt.setStepSize(0.7);
        
        // 设置最大迭代次数
        ndt.setMaximumIterations(40);

        // 设置当前分辨率级别的网格大小
        ndt.setResolution(r);
        
        // 对子地图进行体素降采样，降采样尺寸为分辨率的0.1倍
        rough_map1 = VoxelGrid(submap_kf1, r * 0.1);
        rough_map2 = VoxelGrid(submap_kf2, r * 0.1);
        
        // 设置目标点云（历史关键帧子地图）
        ndt.setInputTarget(rough_map1);
        
        // 设置源点云（当前关键帧点云）
        ndt.setInputSource(rough_map2);

        // 执行配准，使用上一级分辨率的结果作为初始位姿
        ndt.align(*output, Tw2);
        
        // 获取优化后的变换矩阵，作为下一级分辨率的初始值
        Tw2 = ndt.getFinalTransformation();

        // 保存NDT配准得分（配准质量评估）
        c.ndt_score_ = ndt.getTransformationProbability();
    }

    // 将最终变换矩阵转换为double类型
    Mat4d T = Tw2.cast<double>();
    
    // 提取旋转部分（3x3矩阵）并转换为四元数
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();  // 归一化四元数
    
    // 提取平移部分（3x1向量）
    Vec3d t = T.block<3, 1>(0, 3);

    // 计算从kf1到kf2的相对位姿变换
    // Tij = T1^(-1) * T2，其中T2是配准得到的位姿
    c.Tij_ = kf1->GetLIOPose().inverse() * SE3(q, t);

    // 可选：保存配准结果点云，用于调试和可视化
    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_out.pcd", *output);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_tgt.pcd", *rough_map1);
}

/**
 * [功能描述]：位姿图优化，融合里程计约束和回环约束，优化所有关键帧的位姿
 * @return 无返回值
 * 
 * 优化流程：
 * 1. 将当前关键帧添加为位姿图顶点
 * 2. 添加运动约束边（与前面关键帧的相邻约束）
 * 3. 添加回环约束边（与历史关键帧的回环约束）
 * 4. 执行图优化（Levenberg-Marquardt算法）
 * 5. 检测并移除外点（错误的回环约束）
 * 6. 更新所有关键帧的优化位姿
 * 7. 触发回环闭合回调（如重绘地图）
 */
void LoopClosing::PoseOptimization() {
    // 创建SE3位姿顶点，表示当前关键帧在位姿图中的节点
    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(cur_kf_->GetID());
    v->SetEstimate(cur_kf_->GetOptPose());

    // 将顶点添加到优化器
    optimizer_->AddVertex(v);
    
    // 将顶点保存到列表中，用于后续结果提取
    kf_vert_.emplace_back(v);

    /// 添加运动约束边：将当前关键帧与前面的关键帧连接
    /// TODO 3D激光最好是跟前面多个帧都有关联，提高优化的稳定性

    // 与前2个关键帧建立运动约束
    for (int i = 1; i < 3; i++) {
        int id = cur_kf_->GetID() - i;
        
        // 检查ID有效性
        if (id >= 0) {
            auto last_kf = all_keyframes_[id];
            
            // 创建SE3约束边
            auto e = std::make_shared<miao::EdgeSE3>();
            
            // 设置边的两个顶点：前一关键帧和当前关键帧
            e->SetVertex(0, optimizer_->GetVertex(last_kf->GetID()));
            e->SetVertex(1, v);

            // 计算相对运动（从前一帧到当前帧的相对位姿变换）
            // motion = T_last^(-1) * T_cur
            SE3 motion = last_kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
            
            // 设置边的测量值（相对位姿）
            e->SetMeasurement(motion);
            
            // 设置信息矩阵（约束的权重），权重与运动噪声相关
            e->SetInformation(info_motion_);
            
            // 将运动约束边添加到优化器
            optimizer_->AddEdge(e);
        }
    }

    /// 添加回环约束边：将检测到的回环候选添加为约束
    for (auto& c : candidates_) {
        // 创建回环约束边
        auto e = std::make_shared<miao::EdgeSE3>();
        
        // 设置边的两个顶点：历史关键帧和当前关键帧
        e->SetVertex(0, optimizer_->GetVertex(c.idx1_));
        e->SetVertex(1, optimizer_->GetVertex(c.idx2_));
        
        // 设置边的测量值（通过NDT配准得到的相对位姿）
        e->SetMeasurement(c.Tij_);
        
        // 设置信息矩阵（回环约束的权重），通常高于运动约束
        e->SetInformation(info_loops_);

        // 设置鲁棒核函数（Cauchy核），用于抑制错误回环的影响
        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.rk_loop_th_);  // 设置核函数阈值
        e->SetRobustKernel(rk);

        // 将回环约束边添加到优化器
        optimizer_->AddEdge(e);
        
        // 保存回环边，用于后续外点检测
        edge_loops_.emplace_back(e);
    }

    // 如果没有边（约束），无法优化，直接返回
    if (optimizer_->GetEdges().empty()) {
        return;
    }

    // 如果没有回环候选，跳过优化（仅有运动约束无需优化）
    if (candidates_.empty()) {
        return;
    }

    // 初始化优化器，构建优化问题
    optimizer_->InitializeOptimization();
    
    // 关闭详细输出，避免过多日志
    optimizer_->SetVerbose(false);

    // 执行图优化，迭代20次
    optimizer_->Optimize(20);

    /// 移除外点（错误的回环约束）
    int cnt_outliers = 0;
    
    // 遍历所有回环边，检测外点
    for (auto& e : edge_loops_) {
        // 跳过没有鲁棒核的边
        if (e->GetRobustKernel() == nullptr) {
            continue;
        }

        // 检查边的卡方误差是否超过鲁棒核阈值
        // Chi2误差大表示该约束与其他约束不一致，可能是错误的回环
        if (e->Chi2() > e->GetRobustKernel()->Delta()) {
            // 设置边为level 1，在优化中被忽略
            e->SetLevel(1);
            cnt_outliers++;
        } else {
            // 如果约束可靠，移除鲁棒核，使用原始的二次误差
            e->SetRobustKernel(nullptr);
        }
    }

    // 输出外点统计信息
    if (options_.verbose_) {
        LOG(INFO) << "loop outliers: " << cnt_outliers << "/" << edge_loops_.size();
    }

    /// 从优化器中提取优化结果，更新所有关键帧的位姿
    for (auto& vert : kf_vert_) {
        // 获取优化后的位姿
        SE3 pose = vert->Estimate();
        
        // 更新对应关键帧的优化位姿
        all_keyframes_[vert->GetId()]->SetOptPose(pose);
    }

    // 如果设置了回环闭合回调函数，触发回调
    // 回调函数通常用于重绘全局地图或更新可视化
    if (loop_cb_) {
        loop_cb_();
    }

    // 输出优化完成信息，显示回环约束数量
    LOG(INFO) << "optimize finished, loops: " << edge_loops_.size();

    // 可选：输出当前关键帧的优化位姿和LIO位姿对比，用于调试
    // LOG(INFO) << "lc: cur kf " << cur_kf_->GetID() << ", opt: " << cur_kf_->GetOptPose().translation().transpose()
    //           << ", lio: " << cur_kf_->GetLIOPose().translation().transpose();
}

}  // namespace lightning