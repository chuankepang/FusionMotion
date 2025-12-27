/**
 * @file complex_task_manager.cpp
 * @brief 实时模式 - 多阶段任务管理架构（S规划 + 任务状态机）
 */

#include <iostream>
#include <cmath>
#include <thread>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "rokae/robot.h"
#include "rokae/print_helper.hpp"

using namespace rokae;

// =============================================================================
// 1. 实时任务抽象基类
// =============================================================================
class RTTask {
public:
    virtual ~RTTask() = default;
    // 任务启动时的初始化逻辑
    virtual void onStart(const std::array<double, 16>& currentPose) = 0;
    // 每 1ms 执行一次的计算逻辑，返回当前周期的指令位姿
    virtual CartesianPosition update(double dt) = 0;
    // 任务是否结束的判断条件
    virtual bool isFinished() const = 0;
    // 任务名称（调试用）
    virtual std::string name() const = 0;
};

// =============================================================================
// 2. 笛卡尔空间 S 曲线直线运动任务 (API 封装)
// =============================================================================
class SLineTask : public RTTask {
public:
    SLineTask(const std::array<double, 16>& target, double v_max = 0.1, std::string taskName = "SLineMove")
        : target_pose_(target), v_limit_(v_max), task_name_(taskName), finished_(false), time_(0) {}

    std::string name() const override { return task_name_; }

    void onStart(const std::array<double, 16>& currentPose) override {
        start_pose_ = currentPose;
        time_ = 0;
        finished_ = false;

        // 计算距离
        Eigen::Vector3d p1(start_pose_[3], start_pose_[7], start_pose_[11]);
        Eigen::Vector3d p2(target_pose_[3], target_pose_[7], target_pose_[11]);
        double dist = (p2 - p1).norm();

        // 初始化 S 曲线生成器
        generator_ = std::make_unique<CartMotionGenerator>(v_limit_, (dist < 0.0001 ? 0.0001 : dist));
        generator_->calculateSynchronizedValues(0);
        
        std::cout << "[" << task_name_ << "] 启动: 目标距离 " << dist << "m" << std::endl;
    }

    CartesianPosition update(double dt) override {
        time_ += dt;
        double delta_s = 0;
        // 计算当前时刻应达到的路径进度 delta_s
        finished_ = generator_->calculateDesiredValues(time_, &delta_s);

        Eigen::Vector3d p1(start_pose_[3], start_pose_[7], start_pose_[11]);
        Eigen::Vector3d p2(target_pose_[3], target_pose_[7], target_pose_[11]);
        double total_dist = (p2 - p1).norm();
        if (total_dist < 0.0001) total_dist = 0.0001;

        // 1. 位置线性插值
        Eigen::Vector3d p_cur = p1 + (p2 - p1) * (delta_s / total_dist);

        // 2. 姿态 SLERP 插值
        Eigen::Matrix3d m1, m2;
        m1 << start_pose_[0], start_pose_[1], start_pose_[2], start_pose_[4], start_pose_[5], start_pose_[6], start_pose_[8], start_pose_[9], start_pose_[10];
        m2 << target_pose_[0], target_pose_[1], target_pose_[2], target_pose_[4], target_pose_[5], target_pose_[6], target_pose_[8], target_pose_[9], target_pose_[10];
        Eigen::Quaterniond q1(m1), q2(m2);
        Eigen::Quaterniond q_cur = q1.slerp(delta_s / total_dist, q2);
        Eigen::Matrix3d mat = q_cur.toRotationMatrix();

        // 3. 封装输出指令
        CartesianPosition cmd;
        cmd.pos = { mat(0,0), mat(0,1), mat(0,2), p_cur(0),
                    mat(1,0), mat(1,1), mat(1,2), p_cur(1),
                    mat(2,0), mat(2,1), mat(2,2), p_cur(2),
                    0, 0, 0, 1 };
        
        if (finished_) cmd.setFinished(); 
        return cmd;
    }

    bool isFinished() const override { return finished_; }

private:
    std::array<double, 16> start_pose_, target_pose_;
    std::unique_ptr<CartMotionGenerator> generator_;
    double v_limit_, time_;
    bool finished_;
    std::string task_name_;
};

// =============================================================================
// 3. 复杂任务占位符 (例如：视觉伺服或搜索)
// =============================================================================
class ComplexPhasePlaceholder : public RTTask {
public:
    std::string name() const override { return "ComplexPhase_Search"; }
    void onStart(const std::array<double, 16>& currentPose) override {
        std::cout << "[SearchPhase] 启动：开始执行复杂搜索算法..." << std::endl;
        time_ = 0;
        start_pose_ = currentPose;
    }
    CartesianPosition update(double dt) override {
        time_ += dt;
        CartesianPosition cmd;
        cmd.pos = start_pose_;
        // 模拟：在当前位置进行 2cm 的简谐搜索运动
        cmd.pos[3] += 0.02 * std::sin(2 * M_PI * time_); 
        if (time_ > 5.0) is_done_ = true; // 5秒后模拟完成
        return cmd;
    }
    bool isFinished() const override { return is_done_; }
private:
    bool is_done_ = false;
    double time_ = 0;
    std::array<double, 16> start_pose_;
};

// =============================================================================
// 4. 主程序
// =============================================================================
int main() {
    using namespace std;
    rokae::xMateErProRobot robot;

    try {
        robot.connectToRobot("192.168.0.160", "192.168.0.100");
    } catch(const rokae::Exception &e) {
        cerr << "连接失败: " << e.what() << endl;
        return 0;
    }

    std::error_code ec;
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    robot.setPowerState(true, ec);

    try {
        auto rtCon = robot.getRtMotionController().lock();
        robot.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::tcpPose_m});
        
        // --- 任务编排 ---
        vector<shared_ptr<RTTask>> taskQueue;
        
        // 1. 获取当前初始位姿
        robot.updateRobotState(std::chrono::milliseconds(1));
        std::array<double, 16> init_pose;
        robot.getStateData(RtSupportedFields::tcpPose_m, init_pose);

        // 2. 添加阶段1: 向下运动到预定视觉观测点
        auto posA = init_pose; posA[11] -= 0.15; 
        taskQueue.push_back(make_shared<SLineTask>(posA, 0.1, "Phase1_MoveToView"));

        // 3. 添加阶段2: 执行搜索/伺服 (模拟)
        taskQueue.push_back(make_shared<ComplexPhasePlaceholder>());

        // 4. 添加阶段3: 水平回退
        auto posB = posA; posB[3] += 0.1;
        taskQueue.push_back(make_shared<SLineTask>(posB, 0.05, "Phase3_Retract"));

        // --- 实时管理器状态 ---
        size_t currentTaskIdx = 0;
        bool taskInitialized = false;

        // =====================================================================
        // 核心实时回调逻辑
        // =====================================================================
        std::function<CartesianPosition()> masterCallback = [&]() {
            if (currentTaskIdx >= taskQueue.size()) {
                CartesianPosition endCmd;
                endCmd.setFinished();
                return endCmd;
            }

            auto& currentTask = taskQueue[currentTaskIdx];

            // A. 自动初始化每个新阶段
            if (!taskInitialized) {
                std::array<double, 16> current_actual;
                robot.getStateData(RtSupportedFields::tcpPose_m, current_actual);
                currentTask->onStart(current_actual);
                taskInitialized = true;
            }

            // B. 调用当前任务组件的计算逻辑 (1ms 周期)
            CartesianPosition cmd = currentTask->update(0.001);

            // C. 检查当前阶段是否完成并自动跳转
            if (currentTask->isFinished()) {
                currentTaskIdx++;
                taskInitialized = false; // 触发下一个任务的 onStart
                cout << ">>> 任务阶段切换. 剩余任务数: " << taskQueue.size() - currentTaskIdx << endl;
            }

            return cmd;
        };

        // 运行实时控制
        rtCon->setControlLoop(masterCallback, 0, true);
        rtCon->startMove(RtControllerMode::cartesianPosition);
        
        cout << "开始实时任务链运行..." << endl;
        rtCon->startLoop(true); // 阻塞运行，直到所有任务 Finished
        cout << "所有任务执行完毕。" << endl;

    } catch (const exception &e) {
        cerr << "异常: " << e.what() << endl;
    }

    return 0;
}