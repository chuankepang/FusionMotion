/**
 * @file modular_framework.cpp
 * @brief 视力触项目基础框架 - 模块化重构版
 * @note 基于 ROKAE C++ API，仅改变代码结构，保留原有S型规划直线运动功能
 */

#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>
#include <mutex>

// ROKAE Headers
#include "rokae/robot.h"
#include "../print_helper.hpp"

// Eigen Headers
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace rokae;
using namespace std;

// ==========================================
// 1. 定义辅助工具 (将来可以移到 utils.hpp)
// ==========================================
namespace Utils {
    // 将 std::array<double, 16> 转换为 Eigen::Matrix4d
    Eigen::Matrix4d ArrayToEigen(const std::array<double, 16>& pose) {
        Eigen::Matrix4d mat;
        // 注意：这里默认ROKAE输出是列主序(Column-Major)还是行主序(Row-Major)
        // 假设是行主序，逐行填充。如果实际运行不对，需调整。
        mat << pose[0], pose[1], pose[2], pose[3],
               pose[4], pose[5], pose[6], pose[7],
               pose[8], pose[9], pose[10], pose[11],
               pose[12], pose[13], pose[14], pose[15];
        return mat;
    }

    // 将 Eigen::Matrix4d 转换为 std::array<double, 16>
    std::array<double, 16> EigenToArray(const Eigen::Matrix4d& mat) {
        std::array<double, 16> pose;
        // 展平为一维数组
        int idx = 0;
        for(int i=0; i<4; ++i) {
            for(int j=0; j<4; ++j) {
                pose[idx++] = mat(i, j);
            }
        }
        return pose;
    }
}

// ==========================================
// 2. 核心任务类 (将来扩展视触融合逻辑的主要地方)
// ==========================================
class ConnectorInsertionTask {
public:
    ConnectorInsertionTask(const std::string& robot_ip, const std::string& local_ip) 
        : robot_ip_(robot_ip), local_ip_(local_ip) {
        
        // 预分配内存或初始化变量，防止实时核中动态分配
        current_time_ = 0.0;
        is_planning_initialized_ = false;
    }

    ~ConnectorInsertionTask() = default;

    // 初始化机器人连接与配置
    bool initRobot() {
        try {
            std::error_code ec;
            robot_ptr_ = std::make_unique<rokae::xMateErProRobot>(robot_ip_, local_ip_);
            
            robot_ptr_->setOperateMode(rokae::OperateMode::automatic, ec);
            robot_ptr_->setMotionControlMode(MotionControlMode::RtCommand, ec);
            robot_ptr_->setPowerState(true, ec);
            
            // 获取控制器指针
            rt_con_ = robot_ptr_->getRtMotionController().lock();
            if (!rt_con_) return false;

            // 启动状态接收 (重要: 1ms周期)
            robot_ptr_->startReceiveRobotState(std::chrono::milliseconds(1), 
                {RtSupportedFields::jointPos_m, RtSupportedFields::tcpPose_m});
            
            print(std::cout, "Robot Initialized Successfully.");
            return true;

        } catch (const std::exception& e) {
            print(std::cerr, e.what());
            return false;
        }
    }

    // 执行回零或预备姿态 (非实时阻塞运动)
    void moveToStartPose() {
        std::error_code ec;
        robot_ptr_->updateRobotState(std::chrono::milliseconds(1));
        
        std::array<double, 7> current_jnt;
        robot_ptr_->getStateData(RtSupportedFields::jointPos_m, current_jnt);

        // 目标拖拽位姿 (原代码中的硬编码)
        std::array<double, 7> q_drag = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
        
        print(std::cout, "Moving to Start Position...");
        rt_con_->MoveJ(0.5, current_jnt, q_drag);
        
        // 切换到笛卡尔位置控制模式，准备实时控制
        rt_con_->startMove(RtControllerMode::cartesianPosition);
    }

    // 启动实时控制循环
    void startRealTimeLoop() {
        print(std::cout, "Starting Real-Time Control Loop...");

        // 将成员函数绑定为回调
        // 注意：这里使用了 lambda 捕获 this 指针，以便在回调中访问成员变量
        auto callback = [this]() -> CartesianPosition {
            return this->controlCallback();
        };

        rt_con_->setControlLoop(callback);
        rt_con_->startLoop(true); // true 表示阻塞主线程直到停止
        print(std::cout, "Control Loop Finished.");
    }

private:
    // ==========================================
    // 实时控制核心回调函数 (1ms 周期)
    // ==========================================
    CartesianPosition controlCallback() {
        current_time_ += 0.001; // 模拟时钟，将来建议改为真实时钟差

        // --- 1. 初始化阶段 (仅运行一次) ---
        if (!is_planning_initialized_) {
            initializePlanning();
            is_planning_initialized_ = true;
        }

        // --- 2. 轨迹规划与计算 ---
        CartesianPosition cmd;
        double delta_s = 0.0;

        // 使用成员变量 cart_planner_ (避免重复构造)
        // calculateDesiredValues 返回 true 表示正在运动，false/error 表示结束？
        // ⚠️原代码逻辑：if (!calculate...) 是正在运行，else 是结束。需确认API返回值含义。
        // 假设 API: return 0 or false implies valid step
        
        if (!cart_planner_->calculateDesiredValues(current_time_, &delta_s)) {
            
            // 线性插值计算当前位置
            double ratio = delta_s / path_length_;
            Eigen::Vector3d pos_cur = pos_start_vec_ + pos_delta_vec_ * ratio;
            
            // 球面线性插值 (SLERP) 计算当前姿态
            Eigen::Quaterniond rot_cur = rot_start_quat_.slerp(ratio, rot_end_quat_);
            Eigen::Matrix3d mat_cur = rot_cur.normalized().toRotationMatrix();

            // 构建 4x4 矩阵并输出
            Eigen::Matrix4d trans_cur = Eigen::Matrix4d::Identity();
            trans_cur.block<3,3>(0,0) = mat_cur;
            trans_cur.block<3,1>(0,3) = pos_cur;

            cmd.pos = Utils::EigenToArray(trans_cur);
        } else {
            // 运动结束
            cmd.setFinished();
        }

        return cmd;
    }

    // 规划初始化逻辑 (从回调中分离出来)
    void initializePlanning() {
        // 读取当前位姿作为起点
        std::array<double, 16> init_pose_arr;
        robot_ptr_->getStateData(RtSupportedFields::tcpPose_m, init_pose_arr);
        
        Eigen::Matrix4d start_mat = Utils::ArrayToEigen(init_pose_arr);
        
        // 设定终点 (原代码逻辑：Z轴向下0.2m)
        Eigen::Matrix4d end_mat = start_mat;
        end_mat(2, 3) -= 0.2; // Z轴在 index (2,3)

        // 提取向量与四元数，存入成员变量
        pos_start_vec_ = start_mat.block<3,1>(0,3);
        Eigen::Vector3d pos_end_vec = end_mat.block<3,1>(0,3);
        
        rot_start_quat_ = Eigen::Quaterniond(start_mat.block<3,3>(0,0));
        rot_end_quat_ = Eigen::Quaterniond(end_mat.block<3,3>(0,0));

        // 计算路径参数
        pos_delta_vec_ = pos_end_vec - pos_start_vec_;
        path_length_ = pos_delta_vec_.norm();

        // 初始化 ROKAE 的规划器
        // 注意：CartMotionGenerator 需要动态分配或成员变量化
        cart_planner_ = std::make_unique<CartMotionGenerator>(0.05, path_length_); // 0.05 是最大速度/加速度参数?
        cart_planner_->calculateSynchronizedValues(0);
    }

private:
    // 机器人对象
    std::string robot_ip_;
    std::string local_ip_;
    std::unique_ptr<rokae::xMateErProRobot> robot_ptr_;
    std::shared_ptr<rokae::RtMotionController> rt_con_;

    // 状态与规划相关成员变量 (State Persistence)
    double current_time_;
    bool is_planning_initialized_;
    
    // 轨迹参数 (避免在回调中重复计算)
    double path_length_;
    Eigen::Vector3d pos_start_vec_;
    Eigen::Vector3d pos_delta_vec_;
    Eigen::Quaterniond rot_start_quat_;
    Eigen::Quaterniond rot_end_quat_;

    // 规划器实例
    std::unique_ptr<CartMotionGenerator> cart_planner_;
};

// ==========================================
// 3. 主函数 (Main)
// ==========================================
int main() {
    // 实例化任务对象
    ConnectorInsertionTask task("192.168.0.160", "192.168.0.100");

    // 1. 连接机器人
    if (!task.initRobot()) {
        return -1;
    }

    // 2. 运动到初始位置
    task.moveToStartPose();

    // 3. 执行实时任务
    // 这里未来可以改为 switch-case 状态机来调度不同的任务阶段 (视觉/触觉/插入)
    task.startRealTimeLoop();

    return 0;
}