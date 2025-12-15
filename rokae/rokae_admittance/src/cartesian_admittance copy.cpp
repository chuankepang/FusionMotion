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
#include <functional> // 必须包含这个头文件
#include "rokae/motion_control_rt.h" // 必须包含这个头文件
//#include "rokae/motion_generator.hpp" // 必须包含这个头文件

// ROKAE Headers
#include "rokae/robot.h"
#include "rokae/print_helper.hpp"
// 【注意：这里假定你的编译环境已经包含了所有需要的 ROKAE/Eigen 头文件】
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
class MultiModalDockingTask {
public:

    // 状态管理枚举放在 public 区域，供类内部使用，也供外部读取当前状态
    enum class DockingState {
        IDLE,          // 初始空闲/待机
        VISUAL_APPROACH,    // 阶段1: 视觉伺服/粗接近
        TACTILE_CORRECTION, // 阶段2: 触觉修正/微调
        FORCE_SEARCH,       // 阶段3: 寻孔 (力控/位置混合)
        ADMITTANCE_INSERT,  // 阶段4: 柔顺插入
        EXTRACTION_PREP,    // 阶段5: 拔出准备 (插入后的新阶段)
        EXTRACTION_PULL,    // 阶段6: 柔顺拔出
        TASK_FINISHED,      // 任务完成
        TASK_ERROR          // 任务出错
    };

    MultiModalDockingTask(const std::string& robot_ip, const std::string& local_ip) 
        : robot_ip_(robot_ip), local_ip_(local_ip), 
        current_time_(0.0), is_planning_initialized_(false),
        current_state_(DockingState::IDLE) { // 默认初始状态设为 IDLE
        //... 构造函数体
        }

    ~MultiModalDockingTask() = default;

    // 初始化机器人连接与配置
    bool initRobot() {
        try {
            std::error_code ec;
            robot_ptr_ = std::make_unique<rokae::xMateErProRobot>(robot_ip_, local_ip_);
            
            robot_ptr_->setOperateMode(rokae::OperateMode::automatic, ec);
            robot_ptr_->setMotionControlMode(MotionControlMode::RtCommand, ec);
            robot_ptr_->setPowerState(true, ec);
            
            // 获取控制器指针s
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

        // 关键：在启动实时循环前，将状态从 IDLE 切换到第一个运行阶段
        if (current_state_ == DockingState::IDLE) {
            current_state_ = DockingState::VISUAL_APPROACH;
            print(std::cout, "Task started. State: VISUAL_APPROACH");
        }

        print(std::cout, "Starting Real-Time Control Loop...");

        // 1. 将成员函数绑定为 Lambda
        auto lambda_callback = [this]() -> CartesianPosition {
            return this->controlCallback();
        };

        // 2. **【关键修正】** 使用 std::function 显式封装 Lambda 
        std::function<CartesianPosition()> callback = lambda_callback;

        rt_con_->setControlLoop(callback);
        rt_con_->startLoop(true); // true 表示阻塞主线程直到停止
        print(std::cout, "Control Loop Finished.");
    }

private:

// 机器人对象
    std::string robot_ip_;
    std::string local_ip_;

    std::unique_ptr<rokae::xMateErProRobot> robot_ptr_;
    
    // **最终修正类型：使用 API 明确指出的模板类型**
    std::shared_ptr<rokae::RtMotionControlCobot<7>> rt_con_;
    
    double current_time_;
    bool is_planning_initialized_;
    DockingState current_state_;  // 当前任务状态
    
    double path_length_;
    Eigen::Vector3d pos_start_vec_;
    Eigen::Vector3d pos_delta_vec_;
    Eigen::Quaterniond rot_start_quat_;
    Eigen::Quaterniond rot_end_quat_;
    std::unique_ptr<CartMotionGenerator> cart_planner_;

// ==========================================
    // 实时控制核心回调函数 (状态机主循环)
    // ==========================================
    CartesianPosition controlCallback() {
        current_time_ += 0.001; 

        switch (current_state_) {
            case DockingState::VISUAL_APPROACH:
                return handleVisualApproach(); 
            
            case DockingState::TACTILE_CORRECTION:
                return handleTactileCorrection(); 
            
            case DockingState::FORCE_SEARCH:
                return handleForceSearch(); 
            
            case DockingState::ADMITTANCE_INSERT:
                return handleAdmittanceInsert(); 
            
            case DockingState::EXTRACTION_PREP:
                return handleExtractionPrep(); 
                
            case DockingState::EXTRACTION_PULL:
                return handleExtractionPull();
            
            case DockingState::IDLE:
            case DockingState::TASK_ERROR:
                return getKeepPoseCommand(); 
            
            case DockingState::TASK_FINISHED: // **【修正点 1】任务完成状态返回退出指令**
                return getFinalCommand();
            
            default:
                return getKeepPoseCommand();
        }
    }
    
    // 辅助函数：返回带有 setFinished() 标志的指令，用于退出实时循环
    CartesianPosition getFinalCommand() {
        CartesianPosition cmd = getKeepPoseCommand();
        cmd.setFinished(); 
        return cmd;
    }

    // ==========================================
    // ** 核心子函数 1: 视觉粗接近 (S-Line 运动) **
    // ==========================================

    CartesianPosition handleVisualApproach() {
        // --- 1. 初始化检查 ---
        if (!is_planning_initialized_) {
            initializePlanning(); // 初始化规划参数
            is_planning_initialized_ = true;
            print(std::cout, "VISUAL_APPROACH: Planning initialized. Starting move.");
        }

        // --- 2. 轨迹计算 ---
        CartesianPosition cmd = calculateSLineCommand(); 

        // --- 3. 状态切换判断 ---
        // 关键：如果 cmd.isFinished() 为真，说明规划器走完了。
        if (cmd.isFinished()) {
            // S-Line 运动完成，切换到下一个状态
            current_state_ = DockingState::TACTILE_CORRECTION;
            is_planning_initialized_ = false; // 重置初始化标志，为下一阶段准备
            current_time_ = 0.0; // 重置计时器
            print(std::cout, "VISUAL_APPROACH finished. Switching to TACTILE_CORRECTION.");
            
            // ⚠️ 修正点：返回一个“保持当前姿态”的有效指令 (不带 setFinished!)
            return getKeepPoseCommand(); 
        }
        
        // 运动未完成时，返回轨迹计算出的指令
        return cmd;
    }

    // 提取 S-Line 轨迹计算逻辑
    CartesianPosition calculateSLineCommand() {
        CartesianPosition cmd;
        double delta_s = 0.0;
        
        // 注意：这里cart_planner_ 指向的是当前阶段的规划器 (可能是 initializePlanning() 或 initializePlanning_Tactile())
        if (!cart_planner_->calculateDesiredValues(current_time_, &delta_s)) {
            // 运动中
            double ratio = delta_s / path_length_;
            Eigen::Vector3d pos_cur = pos_start_vec_ + pos_delta_vec_ * ratio;
            
            Eigen::Quaterniond rot_cur = rot_start_quat_.slerp(ratio, rot_end_quat_);
            Eigen::Matrix3d mat_cur = rot_cur.normalized().toRotationMatrix();

            Eigen::Matrix4d trans_cur = Eigen::Matrix4d::Identity();
            trans_cur.block<3,3>(0,0) = mat_cur;
            trans_cur.block<3,1>(0,3) = pos_cur;

            cmd.pos = Utils::EigenToArray(trans_cur);
            // 运动中，不设置 Finished 标志
        } else {
            // 规划结束，返回一个设置了 Finished 标志的指令给 handleVisualApproach/handleTactileCorrection
            // 它们会根据这个标志来切换状态，而不是退出 loop。
            CartesianPosition finished_cmd = getKeepPoseCommand(); 
            finished_cmd.setFinished(); // <-- 💥 关键：设置标志给上层函数
            return finished_cmd;
        }
        return cmd;
    }

    // 规划初始化逻辑 (第一段运动: Z轴向下 0.2m)
    void initializePlanning() {
        // 读取当前位姿作为起点
        std::array<double, 16> init_pose_arr;
        robot_ptr_->getStateData(RtSupportedFields::tcpPose_m, init_pose_arr);
        
        Eigen::Matrix4d start_mat = Utils::ArrayToEigen(init_pose_arr);
        
        // 设定终点 (Z轴向下0.2m)
        Eigen::Matrix4d end_mat = start_mat;
        end_mat(2, 3) -= 0.2; 

        // 提取向量与四元数，存入成员变量
        pos_start_vec_ = start_mat.block<3,1>(0,3);
        Eigen::Vector3d pos_end_vec = end_mat.block<3,1>(0,3);
        
        rot_start_quat_ = Eigen::Quaterniond(start_mat.block<3,3>(0,0));
        rot_end_quat_ = Eigen::Quaterniond(end_mat.block<3,3>(0,0));

        // 计算路径参数
        pos_delta_vec_ = pos_end_vec - pos_start_vec_;
        path_length_ = pos_delta_vec_.norm();

        // 初始化 ROKAE 的规划器
        cart_planner_ = std::make_unique<CartMotionGenerator>(0.05, path_length_); 
        cart_planner_->calculateSynchronizedValues(0);
        current_time_ = 0.0; // 确保时间归零
    }
    
    // ==========================================
    // ** 【新增函数】 第二段运动的规划初始化 (Y轴移动 10cm) **
    // ==========================================
    void initializePlanning_Tactile() {
        // 读取当前位姿作为起点 (即上一段运动的终点)
        std::array<double, 16> init_pose_arr;
        robot_ptr_->getStateData(RtSupportedFields::tcpPose_m, init_pose_arr);
        
        Eigen::Matrix4d start_mat = Utils::ArrayToEigen(init_pose_arr);
        
        // 设定终点：在当前位姿基础上，沿Y轴移动 0.1m
        Eigen::Matrix4d end_mat = start_mat;
        end_mat(2, 3) += 0.2; // ** 修正：Y轴移动 10cm (Y轴是索引 1)**

        // 提取向量与四元数，存入成员变量
        pos_start_vec_ = start_mat.block<3,1>(0,3);
        Eigen::Vector3d pos_end_vec = end_mat.block<3,1>(0,3);
        
        rot_start_quat_ = Eigen::Quaterniond(start_mat.block<3,3>(0,0));
        rot_end_quat_ = Eigen::Quaterniond(end_mat.block<3,3>(0,0));

        // 计算路径参数
        pos_delta_vec_ = pos_end_vec - pos_start_vec_;
        path_length_ = pos_delta_vec_.norm();

        // 初始化 ROKAE 的规划器
        cart_planner_ = std::make_unique<CartMotionGenerator>(0.05, path_length_); 
        cart_planner_->calculateSynchronizedValues(0);
        current_time_ = 0.0; // 确保时间归零
    }
    
    // ==========================================
    // ** 【修改函数】 核心子函数 2: 触觉修正 (10cm S-Line 运动) **
    // ==========================================
    CartesianPosition handleTactileCorrection() { 
        // --- 1. 初始化检查 ---
        if (!is_planning_initialized_) {
            initializePlanning_Tactile(); // 初始化第二段运动规划
            is_planning_initialized_ = true;
            print(std::cout, "TACTILE_CORRECTION: Planning initialized. Starting 10cm Y-move.");
        }

        // --- 2. 轨迹计算 ---
        CartesianPosition cmd = calculateSLineCommand(); 

        // --- 3. 状态切换判断 ---
        if (cmd.isFinished()) {
            // S-Line 运动完成，切换到下一个状态
            current_state_ = DockingState::FORCE_SEARCH;
            is_planning_initialized_ = false; // 重置初始化标志
            current_time_ = 0.0; // 重置计时器
            print(std::cout, "TACTILE_CORRECTION finished. Switching to FORCE_SEARCH.");
            
            // 状态切换时返回 Keep Pose
            return getKeepPoseCommand();
        }
        
        return cmd;
    }
    
    CartesianPosition handleForceSearch() { 
        // 逻辑：切换到力控模式 (RtControllerMode::cartesianImpedance)
        // -> 执行螺旋搜索算法
        // -> 检测到入孔力/位移突变
        
        current_state_ = DockingState::ADMITTANCE_INSERT;
        print(std::cout, "FORCE_SEARCH finished. Switching to ADMITTANCE_INSERT.");
        return getKeepPoseCommand();
    }
    
    CartesianPosition handleAdmittanceInsert() { 
        // 逻辑：执行分阶段变参数导纳控制
        // -> 检测到插到底部
        
        current_state_ = DockingState::TASK_FINISHED;
        print(std::cout, "ADMITTANCE_INSERT finished. Switching to TASK_FINISHED.");
        return getKeepPoseCommand();
    }
    
    CartesianPosition handleExtractionPrep() {
        // 逻辑：可能需要做短距离抬升或角度调整，以便分离
        current_state_ = DockingState::EXTRACTION_PULL;
        return getKeepPoseCommand();
    }

    CartesianPosition handleExtractionPull() {
        // 逻辑：柔顺拔出，Z轴施加负向推力或恒定位移
        current_state_ = DockingState::TASK_FINISHED;
        return getKeepPoseCommand();
    }

    // 辅助函数：返回当前位姿指令，使机器人静止
    CartesianPosition getKeepPoseCommand() {
        std::array<double, 16> current_pose_arr;
        // 尝试读取当前位姿，如果读取失败，返回一个空命令
        if (robot_ptr_->getStateData(RtSupportedFields::tcpPose_m, current_pose_arr)) {
            CartesianPosition cmd;
            cmd.pos = current_pose_arr;
            return cmd;
        } else {
            // 如果无法获取状态数据，返回一个空命令，并触发 ERROR 状态
            CartesianPosition cmd;
            current_state_ = DockingState::TASK_ERROR;
            return cmd; 
        }
    }
};

// ==========================================
// 3. 主函数 (Main)
// ==========================================
int main() {
    // 实例化任务对象
    MultiModalDockingTask task("192.168.0.160", "192.168.0.100");

    // 1. 连接机器人
    if (!task.initRobot()) {
        return -1;
    }

    // 2. 运动到初始位置
    task.moveToStartPose();

    // 3. 执行实时任务
    // 实时循环会一直运行，直到任务状态切换到 TASK_FINISHED，并由 controlCallback 返回 setFinished()
    task.startRealTimeLoop();

    // 4. 程序正常退出，若需要下电等操作，应在这里添加
    
    // 示例：安全下电 (如果需要)
    // std::error_code ec;
    // task.robot_ptr_->setPowerState(false, ec); 

    return 0;
}