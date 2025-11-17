/**
 * @file joint_admittance_control.cpp
 * @brief 实时模式 - 7轴关节空间导纳控制（带目标位置 + 柔顺）
 * @note 适配 Rokae xMate 7 轴机械臂
 */

#include <iostream>
#include <cmath>
#include <thread>
#include <Eigen/Dense>
#include "rokae/robot.h"
#include "rokae/print_helper.hpp"
#include "force_buffer/force_buffer.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cust_msgs/msg/stampfloat32array.hpp>  // 新增：力数据消息

using namespace rokae;
using namespace rokae_manipulation;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

constexpr int DoF = 7;

// 全局变量
std::shared_ptr<ForceBuffer> g_force_buffer = nullptr;
Vector7d g_q_desired = Vector7d::Zero();           // 来自 sent_joints
std::mutex g_q_mutex;
bool g_q_received = false;

// 预设参数（需实测标定）
// Eigen::Matrix3d R_sensor_to_flange;   // 传感器 → 法兰 旋转矩阵
// Eigen::Vector3d p_sensor_in_flange;   // 传感器原点在法兰坐标系中的位置
double m_tool = 0.5;                  // 灵巧手+工具质量 [kg]
// Eigen::Vector3d com_tool_in_sensor;   // 工具质心在传感器坐标系中的位置 [m]

// 在类内（推荐，线程安全）或者 main 开头
Eigen::Matrix3d   R_sensor_to_flange = (Eigen::Matrix3d() << 
    1,  0,  0,
    0, -1,  0,
    0,  0, -1).finished();   // 180°绕X旋转（Z轴反向）

Eigen::Vector3d   p_sensor_in_flange{0.0, 0.0, 0.05};   // 传感器在法兰坐标系中偏移 5cm（沿Z）
Eigen::Vector3d   com_tool_in_sensor{0.0, 0.0, 0.03};   // 工具质心在传感器坐标系中偏移 3cm（沿Z）

// 全局变量（已去重力 + 变换后的外力）
std::array<double, 6> g_F_ext = {0};  // [Fx, Fy, Fz, Mx, My, Mz] @ 末端法兰
std::mutex g_force_mutex;
bool g_force_received = false;

// 目标笛卡尔位姿（来自视觉/上位机）
Vector6d g_x_desired = Vector6d::Zero();
std::mutex g_pose_mutex;
bool g_pose_received = false;

int main() {
  using namespace std;

  // ==================== 1. 初始化 ROS2 ====================
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("cartesian_admittance_node");

  // 正确：声明并启动 ROS 线程
  auto ros_spin_thread = std::thread([node]() { rclcpp::spin(node); });

  // ==================== 2. 连接机器人 ====================
  rokae::xMateErProRobot robot; // 本机地址192.168.0.100
  std::error_code ec;
  try {
    robot.connectToRobot("192.168.0.160", "192.168.0.100");
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
    rclcpp::shutdown();
    if (ros_spin_thread.joinable()) {
                ros_spin_thread.join();   // 正确！
            }
    return 0;
  }

  robot.setOperateMode(OperateMode::automatic, ec);
  robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
  robot.setPowerState(true, ec);
  robot.setRtNetworkTolerance(20, ec);

  g_force_buffer = std::make_shared<ForceBuffer>(10);
  std::thread ros_spin([node]() { rclcpp::spin(node); });

  // --- 订阅 sent_joints (7维) ---
  auto joint_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "sent_joints", 10,
      [node](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == DoF) {
          std::lock_guard<std::mutex> lock(g_q_mutex);
          for (int i = 0; i < DoF; ++i) {
            g_q_desired[i] = msg->data[i];
          }
          g_q_received = true;
          RCLCPP_INFO(node->get_logger(), "Received 7D joint target");
        } else {
          RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                               "sent_joints size=%zu, expected %d", msg->data.size(), DoF);
        }
      });

  // 在 ROS 初始化后，订阅目标笛卡尔位姿
  auto pose_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "sent_pose", 10,
      [node](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          if (msg->data.size() == 6) {
              std::lock_guard<std::mutex> lock(g_pose_mutex);
              for (int i = 0; i < 6; ++i) g_x_desired[i] = msg->data[i];
              g_pose_received = true;
              RCLCPP_INFO(node->get_logger(), "Received target pose [x,y,z,rx,ry,rz]");
          } else {
              RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                  "sent_pose size=%zu, expected 6", msg->data.size());
          }
      });

// --- 订阅 force_data (6维，已去重力) ---
  // auto force_sub = node->create_subscription<cust_msgs::msg::Stampfloat32array>(
  //     "force_data", 10,
  //     [node](const cust_msgs::msg::Stampfloat32array::SharedPtr msg) {
  //       if (msg->data.size() == 6) {
  //         std::lock_guard<std::mutex> lock(g_force_mutex);
  //         std::copy(msg->data.begin(), msg->data.end(), g_force_torque.begin());
  //         g_force_received = true;
  //       } else {
  //         RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
  //                              "force_data size=%zu, expected 6", msg->data.size());
  //       }
  //     });

  auto force_sub = node->create_subscription<cust_msgs::msg::Stampfloat32array>(
      "force_data", 10,
      [node, &robot](const cust_msgs::msg::Stampfloat32array::SharedPtr msg) {
          if (msg->data.size() != 6) {
              RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                  "force_data size=%zu, expected 6", msg->data.size());
              return;
          }

          // --- 1. 读取原始力传感器数据 ---
          Eigen::Vector3d F_raw(msg->data[0], msg->data[1], msg->data[2]);  // [N]
          Eigen::Vector3d T_raw(msg->data[3], msg->data[4], msg->data[5]);  // [Nm]

          // --- 2. 获取当前末端位姿（用于重力方向）---
          std::array<double, 6> pose{};
          robot.getStateData(RtSupportedFields::tcpPoseAbc_m, pose);
          Eigen::Vector3d rpy(pose[3], pose[4], pose[5]);
          Eigen::Matrix3d R_flange = Eigen::Matrix3d(
              Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
          );

          // --- 3. 计算工具重力在传感器坐标系中的力 ---
          Eigen::Vector3d g_world(0, 0, -9.81);
          Eigen::Vector3d g_sensor = R_sensor_to_flange.transpose() * R_flange.transpose() * g_world;
          Eigen::Vector3d F_gravity = m_tool * g_sensor;

          // --- 4. 去重力 ---
          Eigen::Vector3d F_external_sensor = F_raw - F_gravity;

          // --- 5. 重力力矩（质心偏移）---
          Eigen::Vector3d T_gravity = com_tool_in_sensor.cross(F_gravity);
          Eigen::Vector3d T_external_sensor = T_raw - T_gravity;

          // --- 6. 变换到机械臂末端法兰坐标系 ---
          Eigen::Vector3d F_ext_flange = R_sensor_to_flange * F_external_sensor;
          Eigen::Vector3d T_ext_flange = R_sensor_to_flange * T_external_sensor
                                      + p_sensor_in_flange.cross(F_ext_flange);

          // --- 7. 存入全局变量 ---
          std::lock_guard<std::mutex> lock(g_force_mutex);
          g_F_ext[0] = F_ext_flange[0]; g_F_ext[1] = F_ext_flange[1]; g_F_ext[2] = F_ext_flange[2];
          g_F_ext[3] = T_ext_flange[0]; g_F_ext[4] = T_ext_flange[1]; g_F_ext[5] = T_ext_flange[2];
          g_force_received = true;
      });



// ==================== 3. 实时控制循环（笛卡尔空间导纳）===================
try {
    auto rtCon = robot.getRtMotionController().lock();

    // 启动实时状态接收：末端位姿 + 关节角（用于 MoveJ）
    robot.startReceiveRobotState(1ms, {
        RtSupportedFields::tcpPoseAbc_m,
        RtSupportedFields::jointPos_m
    });

    // 获取当前状态
    std::array<double, 6> cur_pose{};
    std::array<double, 7> cur_joints{};
    robot.getStateData(RtSupportedFields::tcpPoseAbc_m, cur_pose);
    robot.getStateData(RtSupportedFields::jointPos_m, cur_joints);

    bool init = true;
    double time = 0.0;
    const double dt = 0.001;

    // 初始拖拽笛卡尔位姿（可被 sent_pose 覆盖）
    std::array<double, 6> x_drag = {0.4, 0.0, 0.3, 0.0, M_PI, 0.0};

    // ==================== 笛卡尔导纳参数 ====================
    Eigen::DiagonalMatrix<double,6> M, D, K;
    M.diagonal() << 1.0, 1.0, 1.0, 0.1, 0.1, 0.1;   // 质量
    D.diagonal() << 80,  80,  80,  20,  20,  20;    // 阻尼
    K.diagonal() << 500, 500, 500, 100, 100, 100;   // 刚度

    Vector6d x_ref = Vector6d::Zero();
    Vector6d x_dot = Vector6d::Zero();

    // ==================== 控制回调（笛卡尔空间导纳）===================
    std::function<rokae::CartesianPosition()> callback = [&, rtCon]() -> rokae::CartesianPosition {
        if (init) {
            robot.getStateData(RtSupportedFields::tcpPoseAbc_m, cur_pose);
            x_ref = Eigen::Map<Vector6d>(cur_pose.data());
            init = false;
        }
        time += dt;

        // --- 1. 获取期望笛卡尔位姿（来自 sent_pose）---
        Vector6d x_d = x_ref;  // 默认保持
        {
            std::lock_guard<std::mutex> lock(g_pose_mutex);
            if (g_pose_received) {
                x_d = g_x_desired;
            }
        }

        // --- 2. 获取当前末端位姿 ---
        std::array<double, 6> x_curr{};
        robot.getStateData(RtSupportedFields::tcpPoseAbc_m, x_curr);
        Vector6d x_meas = Eigen::Map<Vector6d>(x_curr.data());

        // --- 3. 获取真实外部力（已去重力+坐标变换）---
        Vector6d F_ext = Vector6d::Zero();
        {
            std::lock_guard<std::mutex> lock(g_force_mutex);
            if (g_force_received) {
                F_ext = Eigen::Map<Vector6d>(g_F_ext.data());
            }
        }

        // --- 4. 笛卡尔导纳控制律 ---
        Vector6d delta_x = M.inverse() * F_ext * dt;           // 惯性
        delta_x -= D * x_dot * dt;                             // 阻尼
        delta_x -= K * (x_ref - x_d) * dt;                     // 刚度（跟踪目标）

        x_ref += delta_x;
        x_dot = delta_x / dt;

        // --- 5. 构造笛卡尔指令 ---
        CartesianPosition cmd;
        cmd.pos = {x_ref[0], x_ref[1], x_ref[2], x_ref[3], x_ref[4], x_ref[5]};

        if (time > 60.0) {
            cmd.setFinished();
        }
        return cmd;
    };

    // --- MoveJ 到初始关节位姿 ---
    std::array<double, 7> q_start{}, q_drag_joints = {0, M_PI/6, M_PI/3, 0, M_PI/2, 0, 0};
    robot.getStateData(RtSupportedFields::jointPos_m, q_start);
    rtCon->MoveJ(0.3, q_start, q_drag_joints);

    // --- 启动笛卡尔控制 ---
    rtCon->setControlLoop(callback);
    rtCon->startMove(RtControllerMode::cartesianPosition);
    rtCon->startLoop(true);
    print(std::cout, "笛卡尔空间导纳控制已启动");

} catch (const std::exception &e) {
    print(std::cerr, "Control error: ", e.what());
}

  rclcpp::shutdown();
  if (ros_spin_thread.joinable()) {
          ros_spin_thread.join();
      }
  return 0;
}