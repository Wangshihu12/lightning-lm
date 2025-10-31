//
// Created by xiang on 2022/2/15.
//

#include "core/lio/eskf.hpp"

namespace lightning {

void ESKF::Predict(const double& dt, const ESKF::ProcessNoiseType& Q, const Vec3d& gyro, const Vec3d& acce) {
    Eigen::Matrix<double, 24, 1> f_ = x_.get_f(gyro, acce);  // 调用get_f 获取 速度 角速度 加速度
    Eigen::Matrix<double, 24, 23> f_x_ = x_.df_dx(acce);

    Eigen::Matrix<double, 24, 12> f_w_ = x_.df_dw();
    Eigen::Matrix<double, 23, process_noise_dim_> f_w_final;

    NavState x_before = x_;
    x_.oplus(f_, dt);

    F_x1_ = CovType::Identity();

    // set f_x_final
    CovType f_x_final;  // 23x23
    for (auto st : x_.vect_states_) {
        int idx = st.idx_;
        int dim = st.dim_;
        int dof = st.dof_;

        for (int i = 0; i < 23; i++) {
            for (int j = 0; j < dof; j++) {
                f_x_final(idx + j, i) = f_x_(dim + j, i);
            }
        }

        for (int i = 0; i < process_noise_dim_; i++) {
            for (int j = 0; j < dof; j++) {
                f_w_final(idx + j, i) = f_w_(dim + j, i);
            }
        }
    }

    Mat3d res_temp_SO3;
    Vec3d seg_SO3;
    for (auto st : x_.SO3_states_) {
        int idx = st.idx_;
        int dim = st.dim_;
        for (int i = 0; i < 3; i++) {
            seg_SO3(i) = -1 * f_(dim + i) * dt;
        }

        F_x1_.block<3, 3>(idx, idx) = math::exp(seg_SO3, 0.5).matrix();

        res_temp_SO3 = math::A_matrix(seg_SO3);
        for (int i = 0; i < state_dim_; i++) {
            f_x_final.template block<3, 1>(idx, i) = res_temp_SO3 * (f_x_.block<3, 1>(dim, i));
        }

        for (int i = 0; i < process_noise_dim_; i++) {
            f_w_final.template block<3, 1>(idx, i) = res_temp_SO3 * (f_w_.block<3, 1>(dim, i));
        }
    }

    Eigen::Matrix<double, 2, 3> res_temp_S2;
    Vec3d seg_S2;
    for (auto st : x_.S2_states_) {
        int idx = st.idx_;
        int dim = st.dim_;
        for (int i = 0; i < 3; i++) {
            seg_S2(i) = f_(dim + i) * dt;
        }

        SO3 res = math::exp(seg_S2, 0.5f);

        Vec2d vec = Vec2d::Zero();
        Eigen::Matrix<double, 2, 3> Nx = x_.grav_.S2_Nx_yy();
        Eigen::Matrix<double, 3, 2> Mx = x_before.grav_.S2_Mx(vec);

        F_x1_.block<2, 2>(idx, idx) = Nx * res.matrix() * Mx;

        Eigen::Matrix<double, 3, 3> x_before_hat = x_before.grav_.S2_hat();
        res_temp_S2 = -Nx * res.matrix() * x_before_hat * math::A_matrix(seg_S2).transpose();

        for (int i = 0; i < state_dim_; i++) {
            f_x_final.block<2, 1>(idx, i) = res_temp_S2 * (f_x_.block<3, 1>(dim, i));
        }
        for (int i = 0; i < process_noise_dim_; i++) {
            f_w_final.block<2, 1>(idx, i) = res_temp_S2 * (f_w_.block<3, 1>(dim, i));
        }
    }

    F_x1_ += f_x_final * dt;
    P_ = (F_x1_)*P_ * (F_x1_).transpose() + (dt * f_w_final) * Q * (dt * f_w_final).transpose();
}

/**
 * 原版的迭代过程中，收敛次数大于1才会结果，所以需要两次收敛。
 * 在未收敛时，实际上不会计算最近邻，也就回避了一次ObsModel的计算
 * 如果这边对每次迭代都计算最近邻的话，时间明显会变长一些，并不是非常合理。。
 *
 * @param obs
 * @param R
 */
/**
 * [功能描述]：ESKF更新步骤，使用迭代扩展卡尔曼滤波（IEKF）进行观测更新
 * @param obs：观测类型（激光雷达、GPS、轮速计等）
 * @param R：观测噪声方差
 * @return 无返回值
 * 
 * 该函数实现迭代扩展卡尔曼滤波（IEKF）更新：
 * 1. 迭代线性化：在每次迭代中重新计算线性化点
 * 2. 流形上的状态更新：正确处理SO3（旋转）和S2（单位球面）流形
 * 3. Anderson加速：加速迭代收敛
 * 4. 协方差更新：更新状态估计的不确定性
 */
void ESKF::Update(ESKF::ObsType obs, const double& R) {
    // 初始化观测模型状态标志
    custom_obs_model_.valid_ = true;
    custom_obs_model_.converge_ = true;

    // 保存预测后的协方差矩阵，用于后续迭代
    CovType P_propagated = P_;

    // 卡尔曼增益相关矩阵
    Eigen::Matrix<double, 23, 1> K_r;      // K * residual，卡尔曼增益乘以残差
    Eigen::Matrix<double, 23, 23> K_H;     // K * H，卡尔曼增益乘以观测矩阵

    // 当前迭代的状态增量（23维误差状态向量）
    StateVecType dx_current = StateVecType::Zero();

    // 保存迭代的起点状态（固定）和上一次迭代的状态
    NavState start_x = x_;   // 迭代起点，保持不变
    NavState last_x = x_;    // 上一次迭代结果

    // 收敛统计
    int converged_times = 0;           // 连续收敛次数
    double last_lidar_res = 0;         // 上一次的激光雷达残差

    // 残差统计
    double init_res = 0.0;             // 初始残差（第一次迭代的残差）
    static double iterated_num = 0;    // 累计迭代次数（用于统计）
    static double update_num = 0;      // 累计更新次数
    update_num += 1;
    
    // IEKF迭代循环：从-1开始是为了先计算一次初始残差
    for (int i = -1; i < maximum_iter_; i++) {
        // 重置观测有效标志
        custom_obs_model_.valid_ = true;

        /// 根据观测类型调用相应的观测函数
        /// 计算observation function，主要包括：residual_（残差）, h_x_（雅可比矩阵H）
        /// x_ 在每次迭代中都会更新，线性化点也会随之更新（这是IEKF的关键）
        if (obs == ObsType::LIDAR || obs == ObsType::WHEEL_SPEED_AND_LIDAR) {
            // 激光雷达观测：点到平面距离
            lidar_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::WHEEL_SPEED) {
            // 轮速计观测
            wheelspeed_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::ACC_AS_GRAVITY) {
            // 加速度作为重力观测（用于重力方向估计）
            acc_as_gravity_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::GPS) {
            // GPS位置观测
            gps_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::BIAS) {
            // IMU偏差观测
            bias_obs_func_(x_, custom_obs_model_);
        }

        // Anderson加速提前终止检查：如果残差不再减小（甚至增大），则回退并终止
        // 检查残差是否增大超过1%，若是则认为发散，回退到上一状态
        if (use_aa_ && i > -1 && (obs == ObsType::LIDAR || obs == ObsType::WHEEL_SPEED_AND_LIDAR) &&
            custom_obs_model_.lidar_residual_mean_ >= last_lidar_res * 1.01) {
            x_ = last_x;
            break;
        }
        iterated_num += 1;

        // 如果观测无效（如特征点太少），跳过本次迭代
        if (!custom_obs_model_.valid_) {
            continue;
        }

        // 第一次迭代（i=-1）时，记录初始残差
        if (i == -1) {
            init_res = custom_obs_model_.lidar_residual_mean_;
            if (init_res < 1e-9) {
                init_res = 1e-9;  // 避免除零，设置最小值
            }
        }

        // 更新迭代统计信息
        iterations_ = i + 2;  // i从-1开始，所以加2得到实际迭代次数
        final_res_ = custom_obs_model_.lidar_residual_mean_ / init_res;  // 相对残差（归一化）

        // 获取观测维度（有效特征点数量）
        int dof_measurement = custom_obs_model_.h_x_.rows();
        
        // 计算当前状态相对于起点的增量：dx = x ⊟ start_x（流形上的减法）
        StateVecType dx = x_.boxminus(start_x);
        dx_current = dx;

        // 重置协方差矩阵为预测值（每次迭代都从预测协方差开始）
        P_ = P_propagated;

        /// 流形上的状态更新：处理SO3和S2流形
        /// 更新P和dx，使用流形上的雅可比矩阵
        /// P = J*P*J^T，dx = J * dx
        
        // 处理所有SO3状态（旋转）：旋转不是线性空间，需要特殊处理
        for (auto it : x_.SO3_states_) {
            int idx = it.idx_;  // 旋转在状态向量中的起始索引
            
            // 提取旋转增量（3维向量）
            Vec3d seg_SO3 = dx.block<3, 1>(idx, 0);
            
            // 计算SO3流形上的雅可比矩阵：A(φ)^T
            // A矩阵用于将切空间（李代数）映射回流形（李群）
            Mat3d res_temp_SO3 = math::A_matrix(seg_SO3).transpose();

            // 更新dx中的旋转部分：dx' = J * dx
            dx_current.block<3, 1>(idx, 0) = res_temp_SO3 * dx.block<3, 1>(idx, 0);

            /// 更新P矩阵中与SO3相关的行：P' = J * P
            for (int j = 0; j < state_dim_; j++) {
                P_.block<3, 1>(idx, j) = res_temp_SO3 * (P_.block<3, 1>(idx, j));
            }
            
            /// 更新P矩阵中与SO3相关的列：P' = P * J^T
            for (int j = 0; j < state_dim_; j++) {
                P_.block<1, 3>(j, idx) = (P_.block<1, 3>(j, idx)) * res_temp_SO3.transpose();
            }
        }

        // 处理所有S2状态（单位球面，如重力方向）
        for (auto it : x_.S2_states_) {
            int idx = it.idx_;  // S2在状态向量中的起始索引

            // 提取S2增量（2维参数化）
            Vec2d seg_S2 = dx.block<2, 1>(idx, 0);

            // 计算S2流形上的雅可比矩阵：Nx * Mx
            // Nx: 当前状态x的投影矩阵，Mx: 从起点到当前点的映射矩阵
            Eigen::Matrix<double, 2, 3> Nx = x_.grav_.S2_Nx_yy();
            Eigen::Matrix<double, 3, 2> Mx = start_x.grav_.S2_Mx(seg_S2);
            Mat2d res_temp_S2 = Nx * Mx;

            // 更新dx中的S2部分
            dx_current.block<2, 1>(idx, 0) = res_temp_S2 * dx.block<2, 1>(idx, 0);

            // 更新P矩阵中与S2相关的行
            for (int j = 0; j < state_dim_; j++) {
                P_.block<2, 1>(idx, j) = res_temp_S2 * (P_.block<2, 1>(idx, j));
            }

            // 更新P矩阵中与S2相关的列
            for (int j = 0; j < state_dim_; j++) {
                P_.block<1, 2>(j, idx) = (P_.block<1, 2>(j, idx)) * res_temp_S2.transpose();
            }
        }

        /// 计算卡尔曼增益并更新状态
        /// 根据观测维度和状态维度的关系，选择不同的计算方法
        
        if (state_dim_ > dof_measurement) {
            // 情况1：状态维度 > 观测维度（一般情况）
            // 使用标准的卡尔曼滤波公式
            
            // 将观测雅可比矩阵H扩展到完整状态维度
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(dof_measurement, state_dim_);
            h_x_cur.topLeftCorner(dof_measurement, 12) = custom_obs_model_.h_x_;
            
            // 设置观测噪声协方差矩阵 R
            custom_obs_model_.R_ = R * Eigen::MatrixXd::Identity(dof_measurement, dof_measurement);

            // 计算卡尔曼增益：K = P * H^T * (H * P * H^T + R)^(-1)
            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            
            // 计算 K * residual
            K_r = K * custom_obs_model_.residual_;
            
            // 计算 K * H
            K_H = K * h_x_cur;
        } else {
            // 情况2：观测维度 >= 状态维度（高维观测，如大量激光点）
            // 使用Sherman-Morrison-Woodbury公式的逆形式，计算效率更高
            
            /// 纯雷达观测的快速计算方法
            double R_inv = 1.0 / (R * dof_measurement);

            // 计算 H^T * R^(-1) * H = H^T * H / R
            Eigen::Matrix<double, 12, 12> HTH = custom_obs_model_.h_x_.transpose() * custom_obs_model_.h_x_;

            // 计算 Q = (P^(-1) / R + H^T * H)^(-1)
            // 这是信息滤波器的形式，避免直接求大矩阵的逆
            CovType P_temp = (P_ / R).inverse();  // P^(-1) / R
            P_temp.block<12, 12>(0, 0) += HTH;    // 加上 H^T * H，得到 Q^(-1)
            CovType Q_inv = P_temp.inverse();     // Q = (Q^(-1))^(-1)

            // 计算 K * residual = Q * H^T * residual
            // 这里K是隐式的，不需要显式计算完整的K矩阵
            K_r = Q_inv.template block<23, 12>(0, 0) * custom_obs_model_.h_x_.transpose() * custom_obs_model_.residual_;

            // 计算 K * H = Q * H^T * H
            K_H.setZero();
            K_H.template block<23, 12>(0, 0) = Q_inv.template block<23, 12>(0, 0) * HTH;
        }

        // 计算新的状态增量：dx = K*r + (K*H - I)*dx
        // 这是IEKF的核心更新公式，结合了观测残差和之前的状态增量
        dx_current = K_r + (K_H - Eigen::Matrix<double, 23, 23>::Identity()) * dx_current;

        // 检查状态增量中是否有NaN（数值异常），如果有则终止更新
        for (int j = 0; j < 23; ++j) {
            if (std::isnan(dx_current(j, 0))) {
                return;
            }
        }

        // 使用计算得到的状态增量更新状态
        if (!use_aa_) {
            // 不使用Anderson加速：直接应用状态增量
            // x = x ⊞ dx（流形上的加法）
            x_ = x_.boxplus(dx_current);
        } else {
            // 使用Anderson加速（AA）：加速迭代收敛
            // AA通过结合历史迭代信息来加速收敛
            
            // 先执行标准更新
            x_ = x_.boxplus(dx_current);

            if (i == -1) {
                // 第一次迭代：初始化Anderson加速器
                aa_.init(dx_current);
            } else {
                // 后续迭代：利用AA计算加速后的状态增量
                // 计算从起点到当前状态的总增量
                auto dx_all = x_.boxminus(start_x);
                
                // AA算法计算加速后的增量
                auto new_dx_all = aa_.compute(dx_all);
                
                // 从起点应用加速后的增量
                x_ = start_x.boxplus(new_dx_all);
            }
        }

        // 保存当前状态，用于下次迭代的发散检测
        last_x = x_;

        // 更新上一次的残差
        last_lidar_res = custom_obs_model_.lidar_residual_mean_;
        
        // 检查迭代是否收敛
        custom_obs_model_.converge_ = true;

        // 检查每个状态分量的增量是否小于阈值
        for (int j = 0; j < 23; j++) {
            if (std::fabs(dx_current[j]) > limit_[j]) {
                // 如果任一分量超过阈值，标记为未收敛
                custom_obs_model_.converge_ = false;
                break;
            }
        }

        // 如果收敛，增加收敛计数
        if (custom_obs_model_.converge_) {
            converged_times++;
        }

        // 如果即将达到最大迭代次数但还未收敛，强制标记为收敛
        if (!converged_times && i == maximum_iter_ - 2) {
            custom_obs_model_.converge_ = true;
        }

        // 终止条件：已经收敛或达到最大迭代次数
        if (converged_times > 0 || i == maximum_iter_ - 1) {
            /// 结束迭代，执行最终的协方差更新
            /// 使用Joseph形式更新协方差矩阵：P = (I - KH)P(I - KH)^T + KRK^T
            /// 简化形式：P = L - KH * P，其中 L = P（已经过流形雅可比变换）
            
            L_ = P_;  // 临时保存P矩阵
            Mat3d res_temp_SO3;
            Vec3d seg_SO3;
            
            // 对所有SO3状态应用流形雅可比变换
            for (auto it : x_.SO3_states_) {
                int idx = it.idx_;
                
                // 提取当前迭代的SO3增量
                for (int j = 0; j < 3; j++) {
                    seg_SO3(j) = dx_current(j + idx);
                }

                // 计算SO3雅可比矩阵
                res_temp_SO3 = math::A_matrix(seg_SO3).transpose();
                
                // 更新L矩阵中与SO3相关的行
                for (int j = 0; j < 23; j++) {
                    L_.block<3, 1>(idx, j) = res_temp_SO3 * (P_.block<3, 1>(idx, j));
                }

                // 更新K_H矩阵中与SO3相关的行
                for (int j = 0; j < 15; j++) {
                    K_H.block<3, 1>(idx, j) = res_temp_SO3 * (K_H.block<3, 1>(idx, j));
                }

                // 更新L和P矩阵中与SO3相关的列
                for (int j = 0; j < 23; j++) {
                    L_.block<1, 3>(j, idx) = (L_.block<1, 3>(j, idx)) * res_temp_SO3.transpose();
                    P_.block<1, 3>(j, idx) = (P_.block<1, 3>(j, idx)) * res_temp_SO3.transpose();
                }
            }

            Mat2d res_temp_S2;
            Vec2d seg_S2;
            
            // 对所有S2状态应用流形雅可比变换
            for (auto it : x_.S2_states_) {
                int idx = it.idx_;

                // 提取当前迭代的S2增量
                for (int j = 0; j < 2; j++) {
                    seg_S2(j) = dx_current(j + idx);
                }

                // 计算S2雅可比矩阵
                Eigen::Matrix<double, 2, 3> Nx = x_.grav_.S2_Nx_yy();
                Eigen::Matrix<double, 3, 2> Mx = start_x.grav_.S2_Mx(seg_S2);
                res_temp_S2 = Nx * Mx;

                // 更新L矩阵中与S2相关的行
                for (auto j = 0; j < 23; j++) {
                    L_.block<2, 1>(idx, j) = res_temp_S2 * (P_.block<2, 1>(idx, j));
                }

                // 更新K_H矩阵中与S2相关的行
                for (auto j = 0; j < 15; j++) {
                    K_H.block<2, 1>(idx, j) = res_temp_S2 * (K_H.block<2, 1>(idx, j));
                }

                // 更新L和P矩阵中与S2相关的列
                for (int j = 0; j < 23; j++) {
                    L_.block<1, 2>(j, idx) = (L_.block<1, 2>(j, idx)) * res_temp_S2.transpose();
                    P_.block<1, 2>(j, idx) = (P_.block<1, 2>(j, idx)) * res_temp_S2.transpose();
                }
            }

            // 最终的协方差更新：P = L - K_H * P
            // 这是卡尔曼滤波协方差更新公式的流形版本
            P_ = L_ - K_H.block<23, 15>(0, 0) * P_.template block<15, 23>(0, 0);

            // 跳出迭代循环
            break;
        }
    }
}

}  // namespace lightning