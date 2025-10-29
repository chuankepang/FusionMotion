// main_node.cpp
#include "force_buffer/force_buffer.hpp"
#include "admittance_node/admittance_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <thread>
#include <iostream>

using namespace rokae_manipulation;

class.DeclareParameter
class MainNode : public rclcpp::Node {
public:
    MainNode() : Node("main_node") {
        // === 1. 初始化力缓存（复用）===
        force_buffer_ = std::make_shared<ForceBuffer>(10);

        // === 2. 初始化阻抗控制器（复用）===
        admittance_ctrl_ = std::make_unique<AdmittanceController>("192.168.0.160");

        // === 3. 固定插入起点（基坐标系）===
        start_pose_ = {
            1, 0, 0, 0.50,
            0, 1, 0, 0.00,
            0, 0, 1, 0.35,  // 孔口上方 5cm
            0, 0, 0, 1
        };

        RCLCPP_INFO(this->get_logger(), "Main node ready. Press ENTER to start...");
    }

private:
    std::array<double, 16> start_pose_;
    std::shared_ptr<ForceBuffer> force_buffer_;
    std::unique_ptr<AdmittanceController> admittance_ctrl_;

    // === 主循环：仅监控力 + 触发 ===
    void run() {
        static bool triggered = false;
        if (!triggered && std::cin.rdbuf()->in_avail() > 0) {
            std::cin.ignore();
            triggered = true;
            start_task();
        }

        // 实时打印力
        static int count = 0;
        if (++count % 20 == 0) {
            auto f = force_buffer_->getLatestForce();
            RCLCPP_INFO(this->get_logger(), "Fz = %.2f N", f[2]);
        }
    }

    // === 任务入口：只做两步！===
    void start_task() {
        RCLCPP_INFO(this->get_logger(), "Step 1: Moving to start pose...");
        if (!move_to_start_pose()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to start pose!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Step 2: Starting impedance insertion...");
        std::thread([this]() {
            try {
                // 100% 复用你写好的函数！不改一行！
                admittance_ctrl_->execute_insertion(
                    start_pose_, -0.05, 8.0, 5.0
                );
                RCLCPP_INFO(this->get_logger(), "Insertion finished!");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Insertion error: %s", e.what());
            }
        }).detach();
    }

    // === 你只需实现这个：MoveL 到起点 ===
    bool move_to_start_pose() {
        // TODO: 用 Rokae SDK 实现
        // rt_con->MoveL(1.0, current_pose, start_pose_);
        RCLCPP_INFO(this->get_logger(), "[SIMULATION] Moving to start pose...");
        std::this_thread::sleep_for(std::chrono::seconds(3));
        return true;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainNode>();
    rclcpp::Rate rate(20);  // 20Hz
    while (rclcpp::ok()) {
        node->run();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}