/**
 * @file joint_admittance_node.cpp
 * @brief 7轴实时关节导纳控制（基于 rt_RobotCtrlNode 架构）
 * @note 功能：sent_joints → 目标位置 + 力传感器 → 柔顺控制
 */

#include <iostream>
#include <thread>
#include <mutex>
#include <array>
#include <deque>
#include <cmath>
#include <Eigen/Dense>
#include "rokae/robot.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cust_msgs/msg/stampfloat32array.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using namespace rokae;
using Vector7d = Eigen::Matrix<double, 7, 1>;

constexpr int DoF = 7;
constexpr int DoF_SENSOR = 6;

enum class ControlMode { Idle, JointAdmittance };

class JointAdmittanceNode : public rclcpp::Node {
public:
    JointAdmittanceNode() : Node("joint_admittance_node") {
        std::error_code ec;
        try {
            robot_.connectToRobot("192.168.0.160", "192.168.0.100");
            RCLCPP_INFO(get_logger(), "Connected to robot");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Connect failed: %s", e.what());
            rclcpp::shutdown(); return;
        }

        // === 基础配置 ===
        robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot_.setRtNetworkTolerance(20, ec);
        robot_.setOperateMode(rokae::OperateMode::automatic,ec);
        robot_.setPowerState(true, ec);

        motion_controller_ = robot_.getRtMotionController().lock();
        robot_.startReceiveRobotState(1ms, {
            RtSupportedFields::jointPos_m
        });

        // === 初始化状态 ===
        robot_.getStateData(RtSupportedFields::jointPos_m, cur_joint_);
        last_valid_joint_ = cur_joint_;
        q_ref_ = Vector7d(cur_joint_.data());
        q_dot_ = Vector7d::Zero();

        // === 导纳参数 ===
        M_.diagonal() << 1.0, 1.0, 1.0, 0.5, 0.5, 0.5, 0.1;
        D_.diagonal() << 80,  80,  80,  40,  40,  40,  10;
        K_.diagonal() << 50,  50,  50,  30,  30,  30,  50;

        // === 订阅 ===
        auto qos = rclcpp::QoS(1).best_effort().deadline(1ms);

        joint_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "sent_joints", qos,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                jointCallback(msg);
            });

        force_sub_ = create_subscription<cust_msgs::msg::Stampfloat32array>(
            "force_data", qos,
            [this](const cust_msgs::msg::Stampfloat32array::SharedPtr msg) {
                forceCallback(msg);
            });

        // === 键盘线程 ===
        keyboard_thread_ = std::thread([this]() { keyboardLoop(); });
    }

    ~JointAdmittanceNode() {
        stopControl();
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
        std::error_code ec;
        robot_.setPowerState(false, ec);
    }

private:
    // ==================== 成员变量 ====================
    xMateErProRobot robot_;
    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_sub_;
    rclcpp::Subscription<cust_msgs::msg::Stampfloat32array>::SharedPtr force_sub_;

    std::array<double, 7> cur_joint_{}, last_valid_joint_{};
    std::array<double, 6> last_force_{};

    std::deque<std::array<double, 7>> joint_queue_;
    std::deque<std::array<double, 6>> force_queue_;
    const size_t max_queue_size_ = 5;

    std::mutex joint_mutex_, force_mutex_;

    Vector7d q_ref_, q_dot_;
    Eigen::DiagonalMatrix<double,7> M_, D_, K_;

    bool init_move_done_ = false;
    bool control_started_ = false;
    std::thread control_thread_, keyboard_thread_;
    ControlMode mode_ = ControlMode::Idle;

    std::error_code ec;

    // ==================== 回调函数 ====================
    void jointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != DoF) {
            RCLCPP_WARN(get_logger(), "sent_joints size=%zu, expected %d", msg->data.size(), DoF);
            return;
        }
        std::array<double, 7> pos;
        for (int i = 0; i < DoF; ++i) pos[i] = msg->data[i];

        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (joint_queue_.size() >= max_queue_size_) joint_queue_.pop_front();
        joint_queue_.push_back(pos);
    }

    void forceCallback(const cust_msgs::msg::Stampfloat32array::SharedPtr msg) {
        if (msg->data.size() != 6) return;
        std::array<double, 6> f;
        for (int i = 0; i < 6; ++i) f[i] = msg->data[i];

        std::lock_guard<std::mutex> lock(force_mutex_);
        if (force_queue_.size() >= max_queue_size_) force_queue_.pop_front();
        force_queue_.push_back(f);
        last_force_ = f;
    }

    // ==================== 导纳控制回调（关节空间）===================
    JointPosition admittanceCallback() {
        // --- 1. 实时读取当前关节位置（7维）---
        std::array<double, 7> q_meas{};
        robot_.getStateData(RtSupportedFields::jointPos_m, q_meas);

        // --- 2. 获取目标平衡位置（来自 sent_joints）---
        Vector7d q_d = q_ref_;  // 默认保持当前参考
        {
            std::lock_guard<std::mutex> lock(joint_mutex_);
            if (!joint_queue_.empty()) {
                auto target = joint_queue_.front();
                joint_queue_.pop_front();
                for (int i = 0; i < DoF; ++i) q_d[i] = target[i];
            }
        }

        RCLCPP_INFO(get_logger(), "q_d get");
        // --- 3. 获取真实外部关节力矩（关键！使用官方API）---
        std::array<double, 7> joint_torque_measured{}, external_torque_measured{};
        std::array<double, 3> cart_torque{}, cart_force{};
        std::error_code ec;

        robot_.getEndTorque(FrameType::flange,
                            joint_torque_measured,
                            external_torque_measured,
                            cart_torque,
                            cart_force,
                            ec);

        if (ec) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "getEndTorque failed: %s", ec.message().c_str());
            JointPosition cmd;
            cmd.setFinished();
            return cmd;
        }

        Vector7d tau_ext = Eigen::Map<Vector7d>(external_torque_measured.data());

        RCLCPP_INFO(get_logger(), "tau_ext get");
        // --- 4. 关节空间导纳控制律 ---
        Vector7d delta_q = M_.inverse() * tau_ext * 0.001;        // 惯性项
        delta_q -= D_ * q_dot_ * 0.001;                           // 阻尼项
        delta_q -= K_ * (q_ref_ - q_d) * 0.001;                    // 刚度项（跟踪 sent_joints）

        q_ref_ += delta_q;
        q_dot_ = delta_q / 0.001;

        // --- 5. 构造并返回7维关节指令 ---
        JointPosition cmd;
        cmd.joints.resize(DoF);
        for (int i = 0; i < DoF; ++i) {
            cmd.joints[i] = q_ref_[i];
        }

        return cmd;
    }

    void startAdmittanceControl() {
        if (mode_ != ControlMode::Idle) return;

        mode_ = ControlMode::JointAdmittance;
        RCLCPP_INFO(get_logger(), "Starting Joint Admittance Control");

        // 关键：初始化参考位置为当前实际位置
        robot_.getStateData(RtSupportedFields::jointPos_m, cur_joint_);
        q_ref_ = Vector7d(cur_joint_.data());
        q_dot_.setZero();

        RCLCPP_INFO(get_logger(), "Starting Moving");
        // MoveJ 到当前位姿（防止跳变）
        motion_controller_->MoveJ(0.1, cur_joint_, cur_joint_);

        RCLCPP_INFO(get_logger(), "Starting RT");
        motion_controller_->setControlLoop(
            std::function<rokae::JointPosition()>([this]() -> rokae::JointPosition {
                return this->admittanceCallback();
            })
        );

        RCLCPP_INFO(get_logger(), "cmd get");
        motion_controller_->startMove(RtControllerMode::jointPosition);
        
        control_thread_ = std::thread([this]() {
            try { motion_controller_->startLoop(true); }
            catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what()); }
        });
        control_started_ = true;
    }

    void stopControl() {
        if (control_started_) {
            motion_controller_->stopLoop();
            if (control_thread_.joinable()) control_thread_.join();
            motion_controller_->stopMove();
            control_started_ = false;
        }
        {
            std::lock_guard<std::mutex> l1(joint_mutex_), l2(force_mutex_);
            joint_queue_.clear(); force_queue_.clear();
        }
        robot_.setOperateMode(rokae::OperateMode::automatic, ec);
        mode_ = ControlMode::Idle;
        RCLCPP_INFO(get_logger(), "Control stopped");
    }

    // ==================== 键盘控制 ====================
    int kbhit() {
        struct termios oldt, newt; int ch, oldf;
        tcgetattr(STDIN_FILENO, &oldt); newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        if (ch != EOF) { ungetc(ch, stdin); return 1; }
        return 0;
    }

    void keyboardLoop() {
        while (rclcpp::ok()) {
            if (kbhit()) {
                char c = getchar();
                if (c == 'f') startAdmittanceControl();
                else if (c == 'q') stopControl();
                else RCLCPP_INFO(get_logger(), "Key: 'f' = start, 'q' = stop");
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    std::string modeToString(ControlMode m) {
        return m == ControlMode::Idle ? "Idle" : "JointAdmittance";
    }
};

// ==================== 主函数 ====================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<JointAdmittanceNode>());
    } catch (...) {}
    rclcpp::shutdown();
    return 0;
}