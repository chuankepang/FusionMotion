#include "rokae_manipulation/force_buffer.hpp"  // 包含头文件
#include <rclcpp/rclcpp.hpp>  // 额外包含（如果头文件未全引）

namespace rokae_manipulation {

ForceBuffer::ForceBuffer(int buffer_size)
    : Node("force_node"), buffer_size_(buffer_size) {
    // 创建订阅器：话题 "force_data"，QoS=10
    subscription_ = this->create_subscription<cust_msgs::msg::Stampfloat32array>(
        "force_data",  // 话题名（根据你的力传感器节点调整）
        rclcpp::QoS(10),  // 可靠性中等
        std::bind(&ForceBuffer::force_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(),
                "ForceBuffer initialized. Subscribing to 'force_data'. Buffer size: %d", buffer_size_);
}

void ForceBuffer::force_callback(const cust_msgs::msg::Stampfloat32array::SharedPtr msg) {
    // 验证数据长度
    if (msg->data.size() != 6) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Invalid force data size: %zu (expected 6)", msg->data.size());
        return;
    }

    // 转换为 Eigen 向量（float32 → double）
    Eigen::Vector6d force;
    for (size_t i = 0; i < 6; ++i) {
        force[i] = static_cast<double>(msg->data[i]);
    }

    // 线程安全插入
    {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.emplace_back(msg->header.stamp, force);
        if (static_cast<int>(buffer_.size()) > buffer_size_) {
            buffer_.pop_front();  // 环形缓冲
        }
    }

    // 可选：日志（低频）
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "New force: Fz=%.3f", force[2]);
}

Eigen::Vector6d ForceBuffer::getLatestForce() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty()) {
        return Eigen::Vector6d::Zero();  // 默认零力
    }
    return buffer_.back().second;
}

std::vector<std::pair<rclcpp::Time, Eigen::Vector6d>> ForceBuffer::getForceBuffer() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {buffer_.begin(), buffer_.end()};  // 返回副本
}

}  // namespace rokae_manipulation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rokae_manipulation::ForceBuffer>(10);  // 默认10
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}