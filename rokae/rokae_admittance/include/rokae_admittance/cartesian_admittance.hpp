/**
 * @file cartesian_admittance.hpp
 * @brief 笛卡尔空间导纳控制器（6DOF）
 *        基于 Rokae 实时控制 + 手腕力传感器 + 重力补偿
 *        支持：零力拖动、速度限幅、安全退出
 *
 * @author 你的名字
 * @date 2025
 */

#pragma once

#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <memory>
#include <rokae/robot.h>
#include "force_buffer/force_buffer.hpp"

using namespace rokae;
using namespace Eigen;
using namespace rokae_manipulation;

namespace rokae_admittance {

/**
 * @brief 导纳控制器类
 */
class CartesianAdmittanceController {
public:
    /**
     * @brief 导纳控制参数结构体
     */
    struct Params {
        double payload_mass{1.5};           // kg
        Vector3d cog_offset{0.0, 0.0, 0.08}; // m，传感器原点 → 质心
        Vector3d gravity{0, 0, -9.81};

        Vector3d M_pos{2.0, 2.0, 2.0};       // 虚拟质量
        Vector3d D_pos{80.0, 80.0, 80.0};    // 虚拟阻尼
        Vector3d K_pos{0.0, 0.0, 0.0};       // 虚拟弹簧（0=纯阻尼）

        Vector3d M_rot{0.3, 0.3, 0.3};
        Vector3d D_rot{10.0, 10.0, 10.0};

        double max_lin_vel{0.08};           // m/s
        double max_ang_vel{0.5};            // rad/s
        double runtime{10.0};               // 运行时间（秒）
    };

    /**
     * @brief 构造函数
     * @param force_node 力传感器缓冲节点
     * @param robot Rokae 机器人实例
     * @param params 导纳参数
     */
    CartesianAdmittanceController(
        std::shared_ptr<ForceBuffer> force_node,
        std::shared_ptr<xMateErProRobot> robot,
        const Params& params = Params());

    /**
     * @brief 设置参考位姿（平衡点）
     * @param pos 参考位置（世界坐标系）
     * @param rot 参考旋转矩阵
     */
    void setReference(const Vector3d& pos, const Matrix3d& rot);

    /**
     * @brief 获取当前控制指令
     * @return CartesianPosition 指令
     */
    CartesianPosition getControlCommand();

    /**
     * @brief 是否已完成（超时或 Ctrl+C）
     */
    bool isFinished() const { return finished_.load(); }

    /**
     * @brief 获取当前末端位置
     */
    Vector3d getCurrentPosition() const { return p_curr_; }

    /**
     * @brief 获取当前外力（已补偿）
     */
    Vector3d getExternalForce() const { return F_ext_; }

private:
    // === 参数 ===
    Params params_;

    // === 外部依赖 ===
    std::shared_ptr<ForceBuffer> force_node_;
    std::shared_ptr<xMateErProRobot> robot_;
    std::shared_ptr<RtMotionController> rt_con_;

    // === 状态变量 ===
    Vector3d p_ref_{Vector3d::Zero()};   // 参考位置
    Vector3d v_lin_{Vector3d::Zero()};   // 线性速度
    Vector3d v_ang_{Vector3d::Zero()};   // 角速度
    AngleAxisd aa_ref_;

    Vector3d p_curr_{Vector3d::Zero()};  // 当前位置
    Matrix3d R_curr_{Matrix3d::Identity()};

    Vector3d F_ext_{Vector3d::Zero()};   // 外部力
    Vector3d Tau_ext_{Vector3d::Zero()}; // 外部力矩

    // === 控制状态 ===
    bool init_{true};
    double time_{0.0};
    const double dt_{0.001};
    std::atomic<bool> finished_{false};

    // === 内部函数 ===
    void updateCurrentPose();
    void updateExternalWrench();
    void applyAdmittanceLaw();
    void integrateState();
    void limitVelocity();
    void buildCommand(CartesianPosition& cmd);
};

}  // namespace rokae_admittance