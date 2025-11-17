#include <iostream>
#include <thread>
#include <mutex>
#include <array>
#include <termios.h>
#include <fcntl.h>
#include <deque>
#include <stdexcept>
#include <unistd.h>
#include <cmath>
#include "rokae_rt/robot.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cust_msgs/msg/stampfloat32array.hpp"

using namespace std::chrono_literals;
using namespace rokae;

enum class RokaeControlMode { Idle, Realtime, Compliance, CartesianPose, CartesianVelocity };

class rt_RobotCtrlNode : public rclcpp::Node {
public:
    rt_RobotCtrlNode() : Node("rt_robot_control_node") {
        std::error_code ec;
        try {
            // 连接到机器人
            std::string robot_ip = "192.168.0.160";
            std::string local_ip = "192.168.0.100";
            robot_.connectToRobot(robot_ip, local_ip);
            RCLCPP_INFO(get_logger(), "Connected to robot at %s", robot_ip.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        try {
            // 设置运动控制模式
            robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to set motion control mode: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Motion control mode failed");
            }
            // 设置网络容差
            robot_.setRtNetworkTolerance(20, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to set network tolerance: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Network tolerance failed");
            }
            // 设置操作模式为自动
            robot_.setOperateMode(OperateMode::automatic, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to set operate mode: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Operate mode failed");
            }
            // 上电
            robot_.setPowerState(true, ec);
            if (ec) {
                RCLCPP_ERROR(get_logger(), "Failed to set power state: %s", ec.message().c_str());
                rclcpp::shutdown();
                throw std::runtime_error("Power state failed");
            }

            // 检查电源状态
            PowerState state = robot_.powerState(ec);
            std::string state_str;
            switch (state) {
                case PowerState::on: state_str = "上电"; break;
                case PowerState::off: state_str = "下电"; break;
                case PowerState::unknown: state_str = "未知"; break;
                case PowerState::estop: state_str = "急停"; break;
                case PowerState::gstop: state_str = "安全门打开"; break;
                default: state_str = "无效状态"; break;
            }
            RCLCPP_INFO(get_logger(), "Robot power state: %s", state_str.c_str());
            if (state != PowerState::on) {
                RCLCPP_ERROR(get_logger(), "Robot not powered on, cannot proceed");
                rclcpp::shutdown();
                throw std::runtime_error("Invalid robot state");
            }

            // 初始化运动控制器
            motion_controller_ = robot_.getRtMotionController().lock();
            // 启动状态接收
            robot_.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m, RtSupportedFields::tcpPoseAbc_m});

            // 获取当前关节位置
            std::array<double, 7> cur_pos {};
            robot_.getStateData(RtSupportedFields::jointPos_m, cur_pos);
            last_valid_command_ = cur_pos;
            RCLCPP_INFO(get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f",
                        cur_pos[0], cur_pos[1], cur_pos[2], cur_pos[3], cur_pos[4], cur_pos[5], cur_pos[6]);

            // 获取当前笛卡尔位姿
            std::array<double, 6> cur_pose {};
            robot_.getStateData(RtSupportedFields::tcpPoseAbc_m, cur_pose);
            last_valid_pose_abc_ = cur_pose;
            last_valid_pose_ = convertToCartesianPosition(cur_pose);
            RCLCPP_INFO(get_logger(), "Current cartesian pose: [x: %f, y: %f, z: %f, rx: %f, ry: %f, rz: %f]",
                        cur_pose[0], cur_pose[1], cur_pose[2], cur_pose[3], cur_pose[4], cur_pose[5]);

            // 初始化关节运动
            std::array<double, 7> target_pos = cur_pos;
            motion_controller_->MoveJ(0.5, cur_pos, target_pos);
            RCLCPP_INFO(get_logger(), "Robot joint positions initialized.");
            init_move_completed = true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Initialization failed: %s", e.what());
            rclcpp::shutdown();
            throw;
        }

        // 设置 ROS QoS
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
            .deadline(rclcpp::Duration(1ms));

        // 订阅关节位置
        joint_positions_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "sent_joints", qos,
            std::bind(&rt_RobotCtrlNode::jointPositionCallback, this, std::placeholders::_1));

        // 订阅力/力矩数据
        force_torque_sub_ = create_subscription<cust_msgs::msg::Stampfloat32array>(
            "force_data", qos,
            std::bind(&rt_RobotCtrlNode::forceTorqueCallback, this, std::placeholders::_1));

        // 订阅笛卡尔位姿
        cartesian_pose_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "cartesian_pose", qos,
            std::bind(&rt_RobotCtrlNode::cartesianPoseCallback, this, std::placeholders::_1));

        // 订阅笛卡尔速度
        cartesian_velocity_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "cartesian_velocity", qos,
            std::bind(&rt_RobotCtrlNode::cartesianVelocityCallback, this, std::placeholders::_1));

        // 启动键盘输入线程
        keyboard_thread_ = std::thread([this]() {
            this->keyboard_input_thread();
        });
    }

    ~rt_RobotCtrlNode() {
        // 停止运动
        stopMotion();
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        // 断电
        std::error_code ec;
        robot_.setPowerState(false, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to power off robot: %s", ec.message().c_str());
        }
        RCLCPP_INFO(get_logger(), "Robot powered off.");
    }

private:
    // 将 [x, y, z, rx, ry, rz] 转换为 CartesianPosition
    CartesianPosition convertToCartesianPosition(const std::array<double, 6>& pose_abc) {
        double x = pose_abc[0], y = pose_abc[1], z = pose_abc[2];
        double rx = pose_abc[3], ry = pose_abc[4], rz = pose_abc[5];

        // 计算 ZYX 欧拉角到旋转矩阵
        double cr = cos(rz), sr = sin(rz);
        double cp = cos(ry), sp = sin(ry);
        double cy = cos(rx), sy = sin(rx);
        std::array<double, 16> matrix = {
            cr*cp, cr*sp*sy - sr*cy, cr*sp*cy + sr*sy, x,
            sr*cp, sr*sp*sy + cr*cy, sr*sp*cy - cr*sy, y,
            -sp,   cp*sy,            cp*cy,           z,
            0,     0,                0,               1
        };

        // 初始化 Frame
        Frame frame;
        frame.trans = {x, y, z};
        frame.rpy = {rx, ry, rz};
        frame.pos = matrix;

        // 初始化 CartesianPosition
        CartesianPosition pos;
        pos.data = frame;
        pos.elbow = 0.0; // 默认臂角
        pos.hasElbow = false; // 不使用臂角
        pos.confData = std::vector<int>(8, 0); // 默认轴配置
        pos.external = std::vector<double>{}; // 无外部关节
        return pos;
    }

    // 处理关节位置回调
    void jointPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 7) {
            std::array<double, 7> new_positions;
            for (size_t i = 0; i < 7; i++) {
                new_positions[i] = static_cast<double>(msg->data[i]);
                if (new_positions[i] < -M_PI || new_positions[i] > M_PI) {
                    RCLCPP_WARN(get_logger(), "Joint angle out of range: %f, clamping to [-pi, pi]", new_positions[i]);
                    new_positions[i] = std::clamp(new_positions[i], -M_PI, M_PI);
                }
            }
            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                if (joint_queue_.size() >= max_queue_size_) {
                    joint_queue_.pop_front();
                    RCLCPP_WARN(get_logger(), "Joint queue full, dropping oldest point");
                }
                joint_queue_.push_back(new_positions);
                RCLCPP_INFO(get_logger(), "Joint queue size: %zu", joint_queue_.size());
            }
        } else {
            RCLCPP_WARN(get_logger(), "Received invalid joint data size: %zu", msg->data.size());
        }
    }

    // 处理力/力矩回调
    void forceTorqueCallback(const cust_msgs::msg::Stampfloat32array::SharedPtr msg) {
        if (msg->data.size() == 6) {
            std::array<double, 6> new_force_torque;
            for (size_t i = 0; i < 6; i++) {
                new_force_torque[i] = static_cast<double>(msg->data[i]);
                if (std::abs(new_force_torque[i]) > 1000.0) {
                    RCLCPP_WARN(get_logger(), "Force/torque %zu out of range: %f, clamping to [-1000, 1000]", i, new_force_torque[i]);
                    new_force_torque[i] = std::clamp(new_force_torque[i], -1000.0, 1000.0);
                }
            }
            {
                std::lock_guard<std::mutex> lock(force_torque_mutex_);
                if (force_queue_.size() >= max_queue_size_) {
                    force_queue_.pop_front();
                    RCLCPP_WARN(get_logger(), "Force queue full, dropping oldest point");
                }
                force_queue_.push_back(new_force_torque);
                last_force_torque_ = new_force_torque;
                RCLCPP_INFO(get_logger(), "Force queue size: %zu, data: [Fx: %f, Fy: %f, Fz: %f, Tx: %f, Ty: %f, Tz: %f]",
                            force_queue_.size(),
                            new_force_torque[0], new_force_torque[1], new_force_torque[2],
                            new_force_torque[3], new_force_torque[4], new_force_torque[5]);
            }
        } else {
            RCLCPP_WARN(get_logger(), "Received invalid force data size: %zu", msg->data.size());
        }
    }

    // 处理笛卡尔位姿回调
    void cartesianPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 6) {
            std::array<double, 6> new_pose;
            for (size_t i = 0; i < 6; i++) {
                new_pose[i] = static_cast<double>(msg->data[i]);
                if (i < 3 && std::abs(new_pose[i]) > 2.0) {
                    RCLCPP_WARN(get_logger(), "Cartesian position %zu out of range: %f, clamping to [-2, 2]", i, new_pose[i]);
                    new_pose[i] = std::clamp(new_pose[i], -2.0, 2.0);
                } else if (i >= 3 && std::abs(new_pose[i]) > M_PI) {
                    RCLCPP_WARN(get_logger(), "Cartesian rotation %zu out of range: %f, clamping to [-pi, pi]", i, new_pose[i]);
                    new_pose[i] = std::clamp(new_pose[i], -M_PI, M_PI);
                }
            }
            {
                std::lock_guard<std::mutex> lock(cartesian_pose_mutex_);
                if (pose_queue_.size() >= max_queue_size_) {
                    pose_queue_.pop_front();
                    RCLCPP_WARN(get_logger(), "Pose queue full, dropping oldest point");
                }
                pose_queue_.push_back(new_pose);
                RCLCPP_INFO(get_logger(), "Pose queue size: %zu, data: [x: %f, y: %f, z: %f, rx: %f, ry: %f, rz: %f]",
                            pose_queue_.size(),
                            new_pose[0], new_pose[1], new_pose[2], new_pose[3], new_pose[4], new_pose[5]);
            }
        } else {
            RCLCPP_WARN(get_logger(), "Received invalid cartesian pose size: %zu", msg->data.size());
        }
    }

    // 处理笛卡尔速度回调
    void cartesianVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 6) {
            std::array<double, 6> new_velocity;
            for (size_t i = 0; i < 6; i++) {
                new_velocity[i] = static_cast<double>(msg->data[i]);
                if (i < 3 && std::abs(new_velocity[i]) > 1.0) {
                    RCLCPP_WARN(get_logger(), "Cartesian velocity %zu out of range: %f, clamping to [-1, 1]", i, new_velocity[i]);
                    new_velocity[i] = std::clamp(new_velocity[i], -1.0, 1.0);
                } else if (i >= 3 && std::abs(new_velocity[i]) > M_PI) {
                    RCLCPP_WARN(get_logger(), "Cartesian angular velocity %zu out of range: %f, clamping to [-pi, pi]", i, new_velocity[i]);
                    new_velocity[i] = std::clamp(new_velocity[i], -M_PI, M_PI);
                }
            }
            {
                std::lock_guard<std::mutex> lock(cartesian_velocity_mutex_);
                if (velocity_queue_.size() >= max_queue_size_) {
                    velocity_queue_.pop_front();
                    RCLCPP_WARN(get_logger(), "Velocity queue full, dropping oldest point");
                }
                velocity_queue_.push_back(new_velocity);
                RCLCPP_INFO(get_logger(), "Velocity queue size: %zu, data: [vx: %f, vy: %f, vz: %f, wx: %f, wy: %f, wz: %f]",
                            velocity_queue_.size(),
                            new_velocity[0], new_velocity[1], new_velocity[2],
                            new_velocity[3], new_velocity[4], new_velocity[5]);
            }
        } else {
            RCLCPP_WARN(get_logger(), "Received invalid cartesian velocity size: %zu", msg->data.size());
        }
    }

    // 实时关节控制回调
    JointPosition rokae_callback() {
        bool is_ready_to_move = false;
        bool has_new_command = false;
        std::array<double, 7> current_target_joint_pos_ {};
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            RCLCPP_INFO(get_logger(), "Joint queue size: %zu", joint_queue_.size());
            if (!joint_queue_.empty()) {
                current_target_joint_pos_ = joint_queue_.front();
                joint_queue_.pop_front();
                last_valid_command_ = current_target_joint_pos_;
                has_new_command = true;
                RCLCPP_INFO(get_logger(), "Processing joint position: [%f, %f, %f, %f, %f, %f, %f]",
                            current_target_joint_pos_[0], current_target_joint_pos_[1], current_target_joint_pos_[2],
                            current_target_joint_pos_[3], current_target_joint_pos_[4], current_target_joint_pos_[5],
                            current_target_joint_pos_[6]);
            }
            if (!init_joint_pos_set_ && init_move_completed) {
                init_joint_pos_set_ = true;
            }
            is_ready_to_move = init_joint_pos_set_;
        }

        // 生成关节命令
        JointPosition cmd;
        if (is_ready_to_move && has_new_command) {
            cmd.joints = std::vector<double>(current_target_joint_pos_.begin(), current_target_joint_pos_.end());
        } else if (is_ready_to_move && !has_new_command) {
            cmd.joints = std::vector<double>(last_valid_command_.begin(), last_valid_command_.end());
        } else {
            std::array<double, 7> current_pos {};
            robot_.getStateData(RtSupportedFields::jointPos_m, current_pos);
            cmd.joints = std::vector<double>(current_pos.begin(), current_pos.end());
        }
        for (double& angle : cmd.joints) {
            if (angle < -M_PI || angle > M_PI) {
                RCLCPP_WARN(get_logger(), "Joint angle out of range: %f, clamping to [-pi, pi]", angle);
                angle = std::clamp(angle, -M_PI, M_PI);
            }
        }
        RCLCPP_INFO(get_logger(), "Sending joint command: [%f, %f, %f, %f, %f, %f, %f]",
                    cmd.joints[0], cmd.joints[1], cmd.joints[2], cmd.joints[3],
                    cmd.joints[4], cmd.joints[5], cmd.joints[6]);
        return cmd;
    }

    // 导纳控制回调
    CartesianPosition admittance_callback() {
        std::array<double, 6> current_force_torque;
        bool has_new_force = false;
        {
            std::lock_guard<std::mutex> lock(force_torque_mutex_);
            if (!force_queue_.empty()) {
                current_force_torque = force_queue_.front();
                force_queue_.pop_front();
                has_new_force = true;
                RCLCPP_INFO(get_logger(), "Processing force/torque: [Fx: %f, Fy: %f, Fz: %f, Tx: %f, Ty: %f, Tz: %f]",
                            current_force_torque[0], current_force_torque[1], current_force_torque[2],
                            current_force_torque[3], current_force_torque[4], current_force_torque[5]);
            } else {
                current_force_torque = last_force_torque_;
            }
        }

        // 获取当前位姿
        std::array<double, 6> current_pose {};
        robot_.getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose);

        // 导纳参数
        std::array<double, 6> K = {1000.0, 1000.0, 1000.0, 50.0, 50.0, 50.0}; // 刚度
        std::array<double, 6> D = {100.0, 100.0, 100.0, 10.0, 10.0, 10.0};   // 阻尼
        double dt = 0.001; // 控制周期 1ms

        // 计算速度
        std::array<double, 6> velocity;
        for (size_t i = 0; i < 6; i++) {
            velocity[i] = (current_force_torque[i] - K[i] * current_pose[i]) / D[i];
        }

        // 更新位姿
        std::array<double, 6> new_pose_abc;
        for (size_t i = 0; i < 6; i++) {
            new_pose_abc[i] = current_pose[i] + velocity[i] * dt;
        }

        CartesianPosition new_pose = convertToCartesianPosition(new_pose_abc);

        RCLCPP_INFO(get_logger(), "Admittance: pose=[%f, %f, %f, %f, %f, %f], force=[%f, %f, %f, %f, %f, %f]",
                    new_pose_abc[0], new_pose_abc[1], new_pose_abc[2], new_pose_abc[3], new_pose_abc[4], new_pose_abc[5],
                    current_force_torque[0], current_force_torque[1], current_force_torque[2],
                    current_force_torque[3], current_force_torque[4], current_force_torque[5]);

        last_valid_pose_abc_ = new_pose_abc;
        last_valid_pose_ = new_pose;
        if (has_new_force) {
            last_force_torque_ = current_force_torque;
        }
        return new_pose;
    }

    // 笛卡尔位姿控制回调
    CartesianPosition cartesian_pose_callback() {
        std::array<double, 6> target_pose;
        bool has_new_pose = false;
        {
            std::lock_guard<std::mutex> lock(cartesian_pose_mutex_);
            if (!pose_queue_.empty()) {
                target_pose = pose_queue_.front();
                pose_queue_.pop_front();
                has_new_pose = true;
                RCLCPP_INFO(get_logger(), "Processing cartesian pose: [x: %f, y: %f, z: %f, rx: %f, ry: %f, rz: %f]",
                            target_pose[0], target_pose[1], target_pose[2],
                            target_pose[3], target_pose[4], target_pose[5]);
            } else {
                target_pose = last_valid_pose_abc_;
            }
        }
        CartesianPosition new_pose = convertToCartesianPosition(target_pose);
        if (has_new_pose) {
            last_valid_pose_ = new_pose;
            last_valid_pose_abc_ = target_pose;
        }
        return new_pose;
    }

    // 笛卡尔速度控制回调
    CartesianPosition cartesian_velocity_callback() {
        std::array<double, 6> target_velocity;
        bool has_new_velocity = false;
        {
            std::lock_guard<std::mutex> lock(cartesian_velocity_mutex_);
            if (!velocity_queue_.empty()) {
                target_velocity = velocity_queue_.front();
                velocity_queue_.pop_front();
                has_new_velocity = true;
                RCLCPP_INFO(get_logger(), "Processing cartesian velocity: [vx: %f, vy: %f, vz: %f, wx: %f, wy: %f, wz: %f]",
                            target_velocity[0], target_velocity[1], target_velocity[2],
                            target_velocity[3], target_velocity[4], target_velocity[5]);
            } else {
                target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            }
        }

        // 获取当前位姿
        std::array<double, 6> current_pose {};
        robot_.getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose);

        // 计算新位姿
        double dt = 0.001; // 控制周期 1ms
        std::array<double, 6> new_pose_abc;
        for (size_t i = 0; i < 6; i++) {
            new_pose_abc[i] = current_pose[i] + target_velocity[i] * dt;
        }

        CartesianPosition new_pose = convertToCartesianPosition(new_pose_abc);

        RCLCPP_INFO(get_logger(), "Velocity control: new_pose=[%f, %f, %f, %f, %f, %f], velocity=[%f, %f, %f, %f, %f, %f]",
                    new_pose_abc[0], new_pose_abc[1], new_pose_abc[2], new_pose_abc[3], new_pose_abc[4], new_pose_abc[5],
                    target_velocity[0], target_velocity[1], target_velocity[2],
                    target_velocity[3], target_velocity[4], target_velocity[5]);

        if (has_new_velocity) {
            last_valid_pose_ = new_pose;
            last_valid_pose_abc_ = new_pose_abc;
        }
        return new_pose;
    }

    // 启动实时关节控制
    void startRealtimeControl() {
        if (current_mode_ != RokaeControlMode::Idle) {
            RCLCPP_WARN(get_logger(), "Cannot start Realtime mode, current mode: %s", modeToString(current_mode_).c_str());
            return;
        }
        current_mode_ = RokaeControlMode::Realtime;
        RCLCPP_INFO(get_logger(), "Entering Realtime control mode");

        // 初始化关节位置
        std::array<double, 7> target_pos {}, cur_pos {};
        robot_.getStateData(RtSupportedFields::jointPos_m, cur_pos);
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            target_pos = joint_queue_.empty() ? cur_pos : joint_queue_.front();
            if (!joint_queue_.empty()) {
                joint_queue_.pop_front();
            }
        }
        motion_controller_->MoveJ(0.5, cur_pos, target_pos);
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            init_move_completed = true;
        }
        // 设置控制循环
        motion_controller_->setControlLoop(
            std::function<JointPosition()>(std::bind(&rt_RobotCtrlNode::rokae_callback, this)),
            0, true
        );
        motion_controller_->startMove(RtControllerMode::jointPosition);
        RCLCPP_INFO(get_logger(), "Control loop started.");
        control_thread_ = std::thread([this]() {
            try {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                RCLCPP_INFO(this->get_logger(), "Joint queue is %s before starting loop", joint_queue_.empty() ? "empty" : "not empty");
                if (!joint_queue_.empty()) {
                    const auto& front_point = joint_queue_.front();
                    RCLCPP_INFO(this->get_logger(), "First point in joint_queue_: [%f, %f, %f, %f, %f, %f, %f]",
                                front_point[0], front_point[1], front_point[2], front_point[3],
                                front_point[4], front_point[5], front_point[6]);
                }
                this->motion_controller_->startLoop(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                rclcpp::shutdown();
            }
        });
        control_loop_started_ = true;
    }

    // 启动导纳控制
    void startComplianceControl() {
        if (current_mode_ != RokaeControlMode::Idle) {
            RCLCPP_WARN(get_logger(), "Cannot start Compliance mode, current mode: %s", modeToString(current_mode_).c_str());
            return;
        }
        current_mode_ = RokaeControlMode::Compliance;
        RCLCPP_INFO(get_logger(), "Entering Compliance control mode");

        // 设置手动模式
        std::error_code ec;
        robot_.setOperateMode(OperateMode::manual, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to set manual mode: %s", ec.message().c_str());
            current_mode_ = RokaeControlMode::Idle;
            return;
        }

        // 初始化位姿
        std::array<double, 6> current_pose {};
        robot_.getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose);
        last_valid_pose_abc_ = current_pose;
        last_valid_pose_ = convertToCartesianPosition(current_pose);

        // 设置导纳控制循环
        motion_controller_->setControlLoop(
            std::function<CartesianPosition()>(std::bind(&rt_RobotCtrlNode::admittance_callback, this)),
            0, true
        );
        motion_controller_->startMove(RtControllerMode::cartesianPosition);
        control_thread_ = std::thread([this]() {
            try {
                std::lock_guard<std::mutex> lock(force_torque_mutex_);
                RCLCPP_INFO(this->get_logger(), "Force queue is %s before starting loop", force_queue_.empty() ? "empty" : "not empty");
                if (!force_queue_.empty()) {
                    const auto& front_force = force_queue_.front();
                    RCLCPP_INFO(this->get_logger(), "First force in force_queue_: [Fx: %f, Fy: %f, Fz: %f, Tx: %f, Ty: %f, Tz: %f]",
                                front_force[0], front_force[1], front_force[2],
                                front_force[3], front_force[4], front_force[5]);
                }
                this->motion_controller_->startLoop(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                rclcpp::shutdown();
            }
        });
        control_loop_started_ = true;
        RCLCPP_INFO(get_logger(), "Admittance control loop started.");
    }

    // 启动笛卡尔位姿控制
    void startCartesianPoseControl() {
        if (current_mode_ != RokaeControlMode::Idle) {
            RCLCPP_WARN(get_logger(), "Cannot start Cartesian Pose mode, current mode: %s", modeToString(current_mode_).c_str());
            return;
        }
        current_mode_ = RokaeControlMode::CartesianPose;
        RCLCPP_INFO(get_logger(), "Entering Cartesian Pose control mode");

        // 设置手动模式
        std::error_code ec;
        robot_.setOperateMode(OperateMode::manual, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to set manual mode: %s", ec.message().c_str());
            current_mode_ = RokaeControlMode::Idle;
            return;
        }

        // 初始化位姿
        std::array<double, 6> current_pose {};
        robot_.getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose);
        last_valid_pose_abc_ = current_pose;
        last_valid_pose_ = convertToCartesianPosition(current_pose);

        // 设置位姿控制循环
        motion_controller_->setControlLoop(
            std::function<CartesianPosition()>(std::bind(&rt_RobotCtrlNode::cartesian_pose_callback, this)),
            0, true
        );
        motion_controller_->startMove(RtControllerMode::cartesianPosition);
        control_thread_ = std::thread([this]() {
            try {
                std::lock_guard<std::mutex> lock(cartesian_pose_mutex_);
                RCLCPP_INFO(this->get_logger(), "Pose queue is %s before starting loop", pose_queue_.empty() ? "empty" : "not empty");
                if (!pose_queue_.empty()) {
                    const auto& front_pose = pose_queue_.front();
                    RCLCPP_INFO(this->get_logger(), "First pose in pose_queue_: [x: %f, y: %f, z: %f, rx: %f, ry: %f, rz: %f]",
                                front_pose[0], front_pose[1], front_pose[2],
                                front_pose[3], front_pose[4], front_pose[5]);
                }
                this->motion_controller_->startLoop(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                rclcpp::shutdown();
            }
        });
        control_loop_started_ = true;
        RCLCPP_INFO(get_logger(), "Cartesian Pose control loop started.");
    }

    // 启动笛卡尔速度控制
    void startCartesianVelocityControl() {
        if (current_mode_ != RokaeControlMode::Idle) {
            RCLCPP_WARN(get_logger(), "Cannot start Cartesian Velocity mode, current mode: %s", modeToString(current_mode_).c_str());
            return;
        }
        current_mode_ = RokaeControlMode::CartesianVelocity;
        RCLCPP_INFO(get_logger(), "Entering Cartesian Velocity control mode");

        // 设置手动模式
        std::error_code ec;
        robot_.setOperateMode(OperateMode::manual, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to set manual mode: %s", ec.message().c_str());
            current_mode_ = RokaeControlMode::Idle;
            return;
        }

        // 初始化位姿
        std::array<double, 6> current_pose {};
        robot_.getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose);
        last_valid_pose_abc_ = current_pose;
        last_valid_pose_ = convertToCartesianPosition(current_pose);

        // 设置速度控制循环
        motion_controller_->setControlLoop(
            std::function<CartesianPosition()>(std::bind(&rt_RobotCtrlNode::cartesian_velocity_callback, this)),
            0, true
        );
        motion_controller_->startMove(RtControllerMode::cartesianPosition);
        control_thread_ = std::thread([this]() {
            try {
                std::lock_guard<std::mutex> lock(cartesian_velocity_mutex_);
                RCLCPP_INFO(this->get_logger(), "Velocity queue is %s before starting loop", velocity_queue_.empty() ? "empty" : "not empty");
                if (!velocity_queue_.empty()) {
                    const auto& front_velocity = velocity_queue_.front();
                    RCLCPP_INFO(this->get_logger(), "First velocity in velocity_queue_: [vx: %f, vy: %f, vz: %f, wx: %f, wy: %f, wz: %f]",
                                front_velocity[0], front_velocity[1], front_velocity[2],
                                front_velocity[3], front_velocity[4], front_velocity[5]);
                }
                this->motion_controller_->startLoop(true);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                rclcpp::shutdown();
            }
        });
        control_loop_started_ = true;
        RCLCPP_INFO(get_logger(), "Cartesian Velocity control loop started.");
    }

    // 停止运动
    void stopMotion() {
        if (control_loop_started_) {
            motion_controller_->stopLoop();
            if (control_thread_.joinable()) {
                control_thread_.join();
            }
            control_loop_started_ = false;
        }
        motion_controller_->stopMove();
        {
            std::lock_guard<std::mutex> lock1(joint_positions_mutex_);
            std::lock_guard<std::mutex> lock2(force_torque_mutex_);
            std::lock_guard<std::mutex> lock3(cartesian_pose_mutex_);
            std::lock_guard<std::mutex> lock4(cartesian_velocity_mutex_);
            joint_queue_.clear();
            force_queue_.clear();
            pose_queue_.clear();
            velocity_queue_.clear();
            last_force_torque_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            init_move_completed = false;
            init_joint_pos_set_ = false;
        }
        // 重置为自动模式
        std::error_code ec;
        robot_.setOperateMode(OperateMode::automatic, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to reset operate mode: %s", ec.message().c_str());
        }
        current_mode_ = RokaeControlMode::Idle;
        RCLCPP_INFO(get_logger(), "Motion stopped, mode reset to Idle");
    }

    // 检测键盘输入
    int kbhit() {
        struct termios oldt, newt;
        int ch, oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        if (ch != EOF) {
            ungetc(ch, stdin);
            return 1;
        }
        return 0;
    }

    // 键盘输入线程
    void keyboard_input_thread() {
        while (rclcpp::ok()) {
            if (kbhit()) {
                char ch = getchar();
                switch (ch) {
                    case 'r':
                        startRealtimeControl();
                        break;
                    case 'f':
                        startComplianceControl();
                        break;
                    case 'p':
                        startCartesianPoseControl();
                        break;
                    case 'v':
                        startCartesianVelocityControl();
                        break;
                    case 'q':
                        stopMotion();
                        break;
                    default:
                        RCLCPP_INFO(get_logger(), "Invalid key: %c, available: r (realtime), f (compliance), p (cartesian pose), v (cartesian velocity), q (stop)", ch);
                        break;
                }
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    // 转换控制模式为字符串
    std::string modeToString(RokaeControlMode mode) {
        switch (mode) {
            case RokaeControlMode::Idle: return "Idle";
            case RokaeControlMode::Realtime: return "Realtime";
            case RokaeControlMode::Compliance: return "Compliance";
            case RokaeControlMode::CartesianPose: return "CartesianPose";
            case RokaeControlMode::CartesianVelocity: return "CartesianVelocity";
            default: return "Unknown";
        }
    }

    rokae::xMateErProRobot robot_;
    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_positions_sub_;
    rclcpp::Subscription<cust_msgs::msg::Stampfloat32array>::SharedPtr force_torque_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cartesian_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cartesian_velocity_sub_;
    const std::array<double, 7> zero_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 7> last_valid_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    CartesianPosition last_valid_pose_;
    std::array<double, 6> last_valid_pose_abc_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> last_force_torque_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::mutex joint_positions_mutex_;
    std::mutex force_torque_mutex_;
    std::mutex cartesian_pose_mutex_;
    std::mutex cartesian_velocity_mutex_;
    std::deque<std::array<double, 7>> joint_queue_;
    std::deque<std::array<double, 6>> force_queue_;
    std::deque<std::array<double, 6>> pose_queue_;
    std::deque<std::array<double, 6>> velocity_queue_;
    const size_t max_queue_size_ = 5;
    bool init_joint_pos_set_ = false;
    bool init_move_completed = false;
    bool control_loop_started_ = false;
    std::thread control_thread_;
    std::thread keyboard_thread_;
    RokaeControlMode current_mode_ = RokaeControlMode::Idle;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<rt_RobotCtrlNode>());
    } catch (const std::exception& e) {
        std::cerr << "Node failed: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}