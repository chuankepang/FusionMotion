/**
 * @file joint_admittance_control.hpp
 * @brief 关节空间导纳控制 - 头文件（兼容官方单文件风格）
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 */

#pragma once

#include <rokae/robot.h>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <array>
#include <functional>
#include <memory>

#include "rokae_manipulation/force_buffer/force_buffer.hpp"
#include "../print_helper.hpp"

using namespace rokae;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace rokae_manipulation {

/**
 * @brief 运行关节空间导纳控制（官方风格封装）
 *
 * @param robot_ip 机器人 IP
 * @param local_ip 本机 IP
 * @param buffer_size 力缓冲区大小
 * @return int 0=成功
 */
int run_joint_admittance_control(
    const std::string& robot_ip,
    const std::string& local_ip = "",
    int buffer_size = 10
);

}  // namespace rokae_manipulation