/**
 * @file admittance_node.hpp
 * @brief 柔顺插入执行器头文件
 *
 * 提供 AdmittanceController 类，用于执行 Z 轴恒力柔顺插入
 * 由 main_node.cpp 调用，插入起点位姿由外部传入
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 */

#ifndef ADMITTANCE_NODE_HPP
#define ADMITTANCE_NODE_HPP

#include <array>
#include <string>
#include <memory>
#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace rokae;

/**
 * @class AdmittanceController
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
 * 使用方式：
 *   @code
 *   AdmittanceController ctrl("192.168.0.160");
 *   std::array<double,16> start_pose = {...};
 *   ctrl.execute_insertion(start_pose, -0.05, 10.0, 5.0);
 *   @endcode
 */
class AdmittanceController {
public:
    /**
     * @brief 构造函数：连接机器人，初始化实时模式
     * @param robot_ip  机器人 IP 地址
     * @param local_ip  本地网卡 IP（可选，默认为 robot_ip）
     */
    explicit AdmittanceController(const std::string& robot_ip,
                                   const std::string& local_ip = "");

    /**
     * @brief 执行柔顺插入
     * @param start_pose  插入起点位姿（4x4 列优先齐次矩阵，基坐标系）
     * @param depth       压入深度 (m)，负值表示向下
     * @param duration    插入总时间 (s)
     * @param fz          Z 轴恒力 (N)
     * @return            成功返回 true，失败抛出异常
     */
    bool execute_insertion(const std::array<double, 16>& start_pose,
                           double depth = -0.05,
                           double duration = 10.0,
                           double fz = 5.0);

    // 禁止拷贝，允许移动（资源管理）
    AdmittanceController(const AdmittanceController&) = delete;
    AdmittanceController& operator=(const AdmittanceController&) = delete;
    AdmittanceController(AdmittanceController&&) = default;
    AdmittanceController& operator=(AdmittanceController&&) = default;

    /// @brief 析构函数：安全退出
    ~AdmittanceController() = default;

private:
    xMateErProRobot robot_;
    std::shared_ptr<RtMotionController> rt_con_;
};

#endif // ADMITTANCE_NODE_HPP