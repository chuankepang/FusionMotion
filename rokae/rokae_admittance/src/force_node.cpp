/**
 * @file force_node.cpp
 * @brief ForceBuffer 实现文件（使用 Eigen::VectorXd）
 */

#include "force_buffer/force_buffer.hpp"
#include <Eigen/Dense>

namespace rokae_manipulation {

ForceBuffer::ForceBuffer(int buffer_size)
    : Node("force_buffer"), buffer_size_(buffer_size)
{
    subscription_ = this->create_subscription<cust_msgs::msg::Stampfloat32array>(
        "force_data", 10,
        std::bind(&ForceBuffer::force_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(),
                "ForceBuffer initialized. Buffer size: %d samples (~%.1f ms @ 1kHz)",
                buffer_size_, buffer_size_ * 1.0);
}

void ForceBuffer::force_callback(const cust_msgs::msg::Stampfloat32array::SharedPtr msg)
{
    if (msg->data.size() != 6) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Received force data with size %zu, expected 6", msg->data.size());
        return;
    }

    // 使用 VectorXd（动态大小）
    Eigen::VectorXd force(6);
    for (int i = 0; i < 6; ++i) {
        force[i] = static_cast<double>(msg->data[i]);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.emplace_back(msg->header.stamp, force);
    if (static_cast<int>(buffer_.size()) > buffer_size_) {
        buffer_.pop_front();
    }
}

Eigen::VectorXd ForceBuffer::getLatestForce() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.empty() ? Eigen::VectorXd::Zero(6) : buffer_.back().second;
}

std::vector<std::pair<rclcpp::Time, Eigen::VectorXd>> ForceBuffer::getForceBuffer() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return std::vector<std::pair<rclcpp::Time, Eigen::VectorXd>>(buffer_.begin(), buffer_.end());
}

}  // namespace rokae_manipulation