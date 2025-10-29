/**
 * @file force_buffer.hpp
 * @brief 力传感器数据缓存器（可注入类，非独立节点）
 *
 * 功能：
 *   - 订阅 force_data 话题
 *   - 线程安全缓存最新 N 条力数据
 *   - 提供 getLatestForce() 接口供 main_node 调用
 *
 * 使用方式：
 *   ForceBuffer buffer(10);  // 缓存 10 条
 *   auto force = buffer.getLatestForce();
 */

#ifndef FORCE_BUFFER_HPP
#define FORCE_BUFFER_HPP

#include <rclcpp/rclcpp.hpp>
#include <cust_msgs/msg/stampfloat32array.hpp>
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <utility>

namespace rokae_manipulation {

class ForceBuffer : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param buffer_size 缓存大小（条数）
     */
    explicit ForceBuffer(int buffer_size = 10);

    /**
     * @brief 获取最新力数据（线程安全）
     * @return Eigen::Vector6d [Fx, Fy, Fz, Mx, My, Mz]
     */
    Eigen::Vector6d getLatestForce() const;

    /**
     * @brief 获取完整缓存（带时间戳）
     */
    std::vector<std::pair<rclcpp::Time, Eigen::Vector6d>> getForceBuffer() const;

private:
    void force_callback(const cust_msgs::msg::Stampfloat32array::SharedPtr msg);

    // 缓存：(时间戳, 力向量)
    mutable std::mutex mutex_;
    std::deque<std::pair<rclcpp::Time, Eigen::Vector6d>> buffer_;
    const int buffer_size_;

    rclcpp::Subscription<cust_msgs::msg::Stampfloat32array>::SharedPtr subscription_;
};

}  // namespace rokae_manipulation

#endif // FORCE_BUFFER_HPP