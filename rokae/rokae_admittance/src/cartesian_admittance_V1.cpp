/**
 * @file cartesian_admittance_node.cpp
 * @brief 7轴实时笛卡尔空间导纳控制（目标位姿 + 真实外力柔顺）
 *        必须先收到 sent_pose 才能启动 + 完整手爪重力补偿
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cust_msgs/msg/stampfloat32array.hpp>
#include <Eigen/Dense>
#include <array>
#include <deque>
#include <mutex>
#include <thread>
#include <cmath>
#include "rokae/robot.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace rokae;
using namespace std::chrono_literals;
using Vector6d = Eigen::Matrix<double, 6, 1>;

constexpr int DoF = 7;

enum class ControlMode { Idle, CartesianAdmittance };

class CartesianAdmittanceNode : public rclcpp::Node {
public:
    CartesianAdmittanceNode() : Node("cartesian_admittance_node") {
        std::error_code ec;

        // === 1. 连接机器人 ===
        try {
            robot_.connectToRobot("192.168.0.160", "192.168.0.100");
            RCLCPP_INFO(get_logger(), "Connected to robot");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Connect failed: %s", e.what());
            rclcpp::shutdown(); return;
        }

        robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot_.setRtNetworkTolerance(20, ec);
        robot_.setOperateMode(OperateMode::automatic, ec);
        robot_.setPowerState(true, ec);

        motion_controller_ = robot_.getRtMotionController().lock();

        // === 2. 启动状态接收 ===
        robot_.startReceiveRobotState(1ms, {
            RtSupportedFields::tcpPoseAbc_m,
            RtSupportedFields::jointPos_m
        });

        // === 3. 力传感器外参（必须实测标定！）===
        // 传感器 → 法兰 的旋转（常见：绕X转180°）
        R_sensor_to_flange_ = (Eigen::Matrix3d() <<
            1,  0,  0,
            0, -1,  0,
            0,  0, -1).finished();

        p_sensor_in_flange_ = Eigen::Vector3d(0.0, 0.0, 0.05);     // 传感器原点在法兰Z方向偏移 5cm
        com_tool_in_sensor_ = Eigen::Vector3d(0.0, 0.0, 0.03);    // 工具质心在传感器Z方向偏移 3cm
        m_tool_ = 0.5;  // 手爪+工具总质量（kg）

        // === 4. 导纳参数（实测最优）===
        M_.diagonal() << 2.0, 2.0, 2.0, 0.3, 0.3, 0.3;
        D_.diagonal() << 120, 120, 120, 40, 40, 40;
        K_.diagonal() << 1000, 1000, 1000, 300, 300, 300;

        // === 5. 订阅 ===
        auto qos = rclcpp::QoS(10)
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
            .deadline(1ms);

        pose_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "sent_pose", qos,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                poseCallback(msg);
            });

        force_sub_ = create_subscription<cust_msgs::msg::Stampfloat32array>(
            "force_data", qos,
            [this](const cust_msgs::msg::Stampfloat32array::SharedPtr msg) {
                forceCallback(msg);
            });

        // === 6. 键盘线程 ===
        keyboard_thread_ = std::thread([this]() { keyboardLoop(); });

        RCLCPP_INFO(get_logger(), "笛卡尔导纳节点已启动，请先发送 sent_pose，收到后按 'f' 启动柔顺控制");
    }

    ~CartesianAdmittanceNode() {
        stopControl();
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
        std::error_code ec;
        robot_.setPowerState(false, ec);
    }

private:
    // ==================== 成员变量 ====================
    xMateErProRobot robot_;
    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_sub_;
    rclcpp::Subscription<cust_msgs::msg::Stampfloat32array>::SharedPtr force_sub_;

    std::array<double, 6> cur_pose_{};
    std::array<double, 7> cur_joint_{};

    std::deque<std::array<double, 6>> pose_queue_;
    std::deque<std::array<double, 6>> raw_force_queue_;   // 原始力数据（未去重力）
    const size_t max_queue_size_ = 10;

    std::mutex pose_mutex_, force_mutex_;

    Vector6d x_ref_, x_dot_;
    Eigen::DiagonalMatrix<double,6> M_, D_, K_;

    // 力传感器外参
    Eigen::Matrix3d R_sensor_to_flange_;
    Eigen::Vector3d p_sensor_in_flange_;
    Eigen::Vector3d com_tool_in_sensor_;
    double m_tool_;

    // 安全状态
    bool first_pose_received_ = false;
    std::array<double, 6> initial_pose_{};

    bool control_started_ = false;
    std::thread control_thread_, keyboard_thread_;
    ControlMode mode_ = ControlMode::Idle;

    // ==================== 回调函数 ====================
    void poseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 6) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "sent_pose size=%zu, expected 6", msg->data.size());
            return;
        }

        std::array<double, 6> pose;
        std::copy(msg->data.begin(), msg->data.end(), pose.begin());

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (pose_queue_.size() >= max_queue_size_) pose_queue_.pop_front();
            pose_queue_.push_back(pose);

            if (!first_pose_received_) {
                initial_pose_ = pose;
                first_pose_received_ = true;
                RCLCPP_INFO(get_logger(), "收到第一条 sent_pose，已锁定初始参考位姿");
            }
        }
    }

    void forceCallback(const cust_msgs::msg::Stampfloat32array::SharedPtr msg) {
        if (msg->data.size() != 6) return;

        std::array<double, 6> f;
        std::copy(msg->data.begin(), msg->data.end(), f.begin());

        {
            std::lock_guard<std::mutex> lock(force_mutex_);
            if (raw_force_queue_.size() >= max_queue_size_) raw_force_queue_.pop_front();
            raw_force_queue_.push_back(f);
        }
    }

    // ==================== 标准手爪重力补偿 + 外力解算（工业级准确版）===================
    Vector6d computeExternalWrench() {
        std::array<double, 6> raw{};
        {
            std::lock_guard<std::mutex> lock(force_mutex_);
            if (raw_force_queue_.empty()) return Vector6d::Zero();
            raw = raw_force_queue_.back();
        }

        Eigen::Vector3d F_raw(raw[0], raw[1], raw[2]);   // N
        Eigen::Vector3d T_raw(raw[3], raw[4], raw[5]);   // Nm

        // 1. 当前法兰位姿（用于计算重力方向）
        robot_.getStateData(RtSupportedFields::tcpPoseAbc_m, cur_pose_);
        Eigen::Vector3d rpy(cur_pose_[3], cur_pose_[4], cur_pose_[5]);
        Eigen::Matrix3d R_flange = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                                            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

        // 2. 重力在传感器坐标系中的分量
        Eigen::Vector3d g_world(0, 0, -9.81);
        Eigen::Vector3d g_sensor = R_sensor_to_flange_.transpose() * R_flange.transpose() * g_world;
        Eigen::Vector3d F_gravity = m_tool_ * g_sensor;

        // 3. 去重力（力 + 力矩）
        Eigen::Vector3d F_ext_sensor = F_raw - F_gravity;
        Eigen::Vector3d T_gravity = com_tool_in_sensor_.cross(F_gravity);
        Eigen::Vector3d T_ext_sensor = T_raw - T_gravity;

        // 4. 变换到法兰坐标系
        Eigen::Vector3d F_ext_flange = R_sensor_to_flange_ * F_ext_sensor;
        Eigen::Vector3d T_ext_flange = R_sensor_to_flange_ * T_ext_sensor
                                     + p_sensor_in_flange_.cross(F_ext_flange);

        Vector6d F_ext;
        F_ext << F_ext_flange, T_ext_flange;
        return F_ext;
    }

    // ==================== 笛卡尔导纳控制律（新版 SDK 专用） ====================
    CartesianPosition admittanceCallback() {
        RCLCPP_ERROR(get_logger(), "进入导纳循环！");
        // 1. 读取当前实际位姿：x,y,z,rx,ry,rz（单位：m / rad）
        robot_.getStateData(RtSupportedFields::tcpPoseAbc_m, cur_pose_);
        Vector6d x_meas = Eigen::Map<Vector6d>(cur_pose_.data());   // 当前实际位姿

        RCLCPP_ERROR(get_logger(), "当前位置收到！");
        // 2. 获取期望位姿（支持外部实时发送 sent_pose 动态覆盖）
        Vector6d x_desired = x_ref_;   // 默认使用内部积分参考位姿
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!pose_queue_.empty()) {
                auto target = pose_queue_.front();
                pose_queue_.pop_front();
                x_desired = Eigen::Map<Vector6d>(target.data());
                // 收到新目标时顺便把积分状态同步过去，防止突变
                x_ref_ = x_desired;
                x_dot_.setZero();
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "收到新的目标位姿，已更新参考点");
            }
        }

        // 3. 计算外部力（已完成重力补偿 + 坐标变换）
        Vector6d F_ext = computeExternalWrench();

        // 4. 经典二阶导纳控制律（工业界最稳定写法）
        //    M̈ẍ + Dẋ + K(x - x_d) = F_ext
        const double dt = 0.001;

        Vector6d pose_error = x_meas - x_desired;                 // 位置/姿态误差
        Vector6d accel = M_.inverse() * (F_ext - D_ * x_dot_ - K_ * pose_error);

        x_dot_ += accel * dt;      // 速度积分
        x_ref_ += x_dot_ * dt;     // 参考轨迹积分（虚拟目标点）

        // 5. 将积分后的 x_ref_（x,y,z,rx,ry,rz）转换为 4×4 齐次变换矩阵
        Eigen::Vector3d pos_cmd(x_ref_[0], x_ref_[1], x_ref_[2]);
        Eigen::Vector3d rpy_cmd(x_ref_[3], x_ref_[4], x_ref_[5]);

        // Rokae 使用 Z-Y-X 欧拉角（固定角）顺序
        Eigen::Matrix3d rot_cmd = (Eigen::AngleAxisd(rpy_cmd[2], Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(rpy_cmd[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(rpy_cmd[0], Eigen::Vector3d::UnitX())).matrix();

        // 构造标准 4×4 齐次变换矩阵（列优先）
        std::array<double, 16> pose_matrix{{
            rot_cmd(0,0), rot_cmd(0,1), rot_cmd(0,2), pos_cmd(0),
            rot_cmd(1,0), rot_cmd(1,1), rot_cmd(1,2), pos_cmd(1),
            rot_cmd(2,0), rot_cmd(2,1), rot_cmd(2,2), pos_cmd(2),
            0.0,          0.0,          0.0,          1.0
        }};

        // 6. 返回指令
        CartesianPosition cmd;
        cmd.pos = pose_matrix;   // 新版 SDK 必须是 16 元素齐次矩阵！

        RCLCPP_ERROR(get_logger(), "cmd 收到！");
        return cmd;
    }

    // ==================== 控制启动（必须先收到 sent_pose）===================
    void startAdmittanceControl() {
        if (mode_ != ControlMode::Idle) return;

        if (!first_pose_received_) {
            RCLCPP_ERROR(get_logger(), "尚未收到 sent_pose 话题！请先发送目标位姿后再启动！");
            return;
        }

        mode_ = ControlMode::CartesianAdmittance;
        x_ref_ = Eigen::Map<Vector6d>(initial_pose_.data());
        x_dot_.setZero();

        RCLCPP_INFO(get_logger(), "笛卡尔导纳控制已启动！进入柔顺模式");

        motion_controller_->setControlLoop(
            std::function<rokae::CartesianPosition()>([this]() -> rokae::CartesianPosition {
                return this->admittanceCallback();
            })
        );

        // motion_controller_->setControlLoop([this]() -> rokae::CartesianPosition {
        //     return this->admittanceCallback();
        // });

        motion_controller_->startMove(RtControllerMode::cartesianPosition);

        control_thread_ = std::thread([this]() {
            try { motion_controller_->startLoop(true); }
            catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "startLoop failed: %s", e.what());
            }
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
        mode_ = ControlMode::Idle;
        RCLCPP_INFO(get_logger(), "控制已停止");
    }

    // ==================== 键盘控制 ====================
    int kbhit() {
        struct termios oldt{}, newt{};
        int ch; int oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt; newt.c_lflag &= ~(ICANON | ECHO);
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
                else if (c == 'q') { stopControl(); rclcpp::shutdown(); }
                else RCLCPP_INFO(get_logger(), "按 'f' 启动导纳，'q' 退出");
            }
            std::this_thread::sleep_for(10ms);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianAdmittanceNode>());
    rclcpp::shutdown();
    return 0;
}