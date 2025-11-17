/**
 * @file admittance_node.cpp
 * @brief 柔顺插入执行器（纯执行模块）
 *
 * 功能：
 *   - 接收外部传入的插入起点位姿（4x4 齐次矩阵）
 *   - 执行 Z 轴恒力柔顺压入指定距离
 *   - XY 方向：高刚度 + 期望力 0 → 无偏移、无力
 *   - Z  方向：刚度 0 + 期望力 Fz → 恒力插入
 *
 * 依赖：
 *   - 粗定位由 main_node.cpp 完成
 *   - 负载已通过教学器标定
 *   - 工具坐标系由本模块设置
 *
 * 调用方式：
 *   AdmittanceController ctrl("192.168.0.160");
 *   ctrl.execute_insertion(start_pose_array, -0.05, 10.0, 5.0);
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 */

#include "admittance_node/admittance_node.hpp"
#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>
#include <array>
#include "rokae_rt/robot.h"
#include "rokae_rt/utility.h"

using namespace rokae;

class AdmittanceController {
public:
    /**
     * @brief 构造函数：连接机器人，初始化实时模式
     */
    explicit AdmittanceController(const std::string& robot_ip,
                                   const std::string& local_ip = "")
        : robot_(robot_ip, local_ip.empty() ? robot_ip : local_ip) {
        std::error_code ec;
        robot_.setRtNetworkTolerance(50, ec);
        robot_.setOperateMode(rokae::OperateMode::automatic, ec);
        robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot_.setPowerState(true, ec);

        rt_con_ = robot_.getRtMotionController().lock();
        if (!rt_con_) {
            throw std::runtime_error("[ADMITTANCE] Failed to acquire RtMotionController");
        }
    }

    /**
     * @brief 执行柔顺插入
     * @param start_pose  插入起点位姿（4x4 列优先齐次矩阵，基坐标系）
     * @param depth       压入深度 (m)，负值表示向下
     * @param duration    插入总时间 (s)
     * @param fz          Z 轴恒力 (N)
     * @return            成功返回 true
     */
    bool execute_insertion(const std::array<double, 16>& start_pose,
                           double depth = -0.05,
                           double duration = 10.0,
                           double fz = 5.0) {
        std::error_code ec;

        // === 1. 设置工具坐标系（必须！手爪 TCP）===
        // 假设手爪尖端在法兰坐标系下向前 100mm
        std::array<double, 16> tool_to_flange = {
            1, 0, 0, 0.10,   // X 轴向前 10cm
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        };
        rt_con_->setFcCoor(tool_to_flange, FrameType::tool, ec);

        // === 2. 设置阻抗控制参数（核心）===
        rt_con_->setCartesianImpedance({1500, 1500, 0, 100, 100, 0}, ec);     // XY 高刚度，Z 柔顺
        rt_con_->setCartesianImpedanceDesiredTorque({0, 0, fz, 0, 0, 0}, ec); // Z 恒力，XY 无力

        // === 3. 启动阻抗控制模式 ===
        rt_con_->startMove(RtControllerMode::cartesianImpedance);

        // === 4. 实时控制循环（1kHz）===
        double time = 0.0;
        std::atomic<bool> task_running{true};

        std::function<CartesianPosition(void)> control_loop = [&]() -> CartesianPosition {
            time += 0.001;  // 1ms 步长

            CartesianPosition output{};
            output.pos = start_pose;  // 外部传入的起点

            double delta_z = 0.0;
            if (time <= duration) {
                // 线性压入
                delta_z = depth * (time / duration);

                // 可选：余弦缓动（更柔顺）
                // delta_z = depth * (1 - std::cos(M_PI * time / duration)) / 2.0;
            } else {
                delta_z = depth;
                output.setFinished();
                task_running.store(false);
                std::cout << "[ADMITTANCE] 柔顺插入完成！"
                          << " 深度: " << -depth * 1000 << " mm"
                          << " | 恒力: " << fz << " N"
                          << std::endl;
            }

            output.pos[7] += delta_z;  // 修改 Z 坐标（工具坐标系）
            return output;
        };

        rt_con_->setControlLoop(control_loop);
        rt_con_->startLoop(false);  // 后台运行

        // === 5. 等待任务结束 ===
        while (task_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        rt_con_->stopLoop();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "[ADMITTANCE] 柔顺控制安全退出" << std::endl;
        return true;
    }

private:
    xMateErProRobot robot_;
    std::shared_ptr<RtMotionController> rt_con_;
};