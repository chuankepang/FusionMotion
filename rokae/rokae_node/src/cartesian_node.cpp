// cartesian_tracker_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>           // 如果你发 Pose
// #include <geometry_msgs/msg/pose_stamped.hpp> // 如果带时间戳
#include <array>
#include <deque>
#include <mutex>
#include <thread>
#include "rokae_node/robot.h"
#include "rokae_node/data_types.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace rokae;
using namespace std::chrono_literals;

class CartesianTrackerNode : public rclcpp::Node {
public:
    CartesianTrackerNode() : Node("cartesian_tracker_node") {
        std::error_code ec;

        // === 1. 连接机器人 ===
        try {
            robot_.connectToRobot("192.168.0.160", "192.168.0.100");
            robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
            robot_.setRtNetworkTolerance(20, ec);
            robot_.setOperateMode(OperateMode::automatic, ec);
            robot_.setPowerState(true, ec);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "连接失败: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        motion_controller_ = robot_.getRtMotionController().lock();

        // === 2. 启动状态接收 ===
        robot_.startReceiveRobotState(1ms, {RtSupportedFields::jointPos_m});

        // === 3. 订阅 sent_pose（支持 size=6,7,16）===
        // 新写法（Iron/Jazzy/Rolling 通用，永不报错）
        auto qos = rclcpp::QoS(10)
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        pose_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "sent_pose", qos,
            std::bind(&CartesianTrackerNode::poseCallback, this, std::placeholders::_1));

        // === 4. 控制线程 ===
        control_thread_ = std::thread([this]() { this->controlLoop(); });

        RCLCPP_INFO(this->get_logger(), "笛卡尔跟踪节点已启动，等待 sent_pose...");
    }

    ~CartesianTrackerNode() {
        if (motion_controller_ && loop_started_) {
            motion_controller_->stopLoop();
        }
        if (control_thread_.joinable()) control_thread_.join();
        std::error_code ec;
        robot_.setPowerState(false, ec);
    }

private:
    void poseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::array<double, 16> pose_matrix{};

        if (msg->data.size() == 16) {
            std::copy(msg->data.begin(), msg->data.end(), pose_matrix.begin());
        }
        else if (msg->data.size() == 6) {
            // [x,y,z,rx,ry,rz] → 构造旋转矩阵
            pose_matrix = pose6d_to_matrix(msg->data);
        }
        else if (msg->data.size() == 7) {
            // [x,y,z,qx,qy,qz,qw]
            pose_matrix = pose7d_to_matrix(msg->data);
        }
        else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "sent_pose size错误: %zu (期望6/7/16)", msg->data.size());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (pose_queue_.size() >= 50) pose_queue_.pop_front();
            pose_queue_.push_back(pose_matrix);
        }

        if (!control_started_) {
            control_started_ = true;
            RCLCPP_INFO(this->get_logger(), "收到第一条位姿，准备启动实时笛卡尔控制");
        }
    }

    void controlLoop() {
        while (rclcpp::ok()) {
            CartesianPosition target{};
            bool has_new = false;

            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                if (!pose_queue_.empty()) {
                    target.posture = pose_queue_.front();
                    pose_queue_.pop_front();
                    has_new = true;
                }
            }

            if (!control_started_ || !has_new) {
                std::this_thread::sleep_for(1ms);
                continue;
            }

            // === 关键：只启动一次循环 ===
            if (!loop_started_) {
                motion_controller_->setControlLoop([this, target]() mutable -> CartesianPosition {
                    // 每次都返回最新目标（必须是新对象！）
                    CartesianPosition cmd;
                    {
                        std::lock_guard<std::mutex> lock(pose_mutex_);
                        if (!pose_queue_.empty()) {
                            cmd.posture = pose  // 取出最新的一条
                            pose_queue_.back();  // 或者 front，随你
                        } else {
                            cmd.posture = target.posture;  // 兜底
                        }
                    }
                    return cmd;
                }, 0, true);  // useStateDataInLoop = true

                motion_controller_->startMove(RtControllerMode::cartesianPosition);
                motion_controller_->startLoop(true);  // 阻塞式循环
                loop_started_ = true;
                RCLCPP_INFO(this->get_logger(), "实时笛卡尔控制已启动！");
            }
        }
    }

    std::array<double, 16> pose6d_to_matrix(const std::vector<float>& data) {
        if (data.size() < 6) return std::array<double, 16>{};

        Eigen::Vector3d trans(data[0], data[1], data[2]);
        Eigen::Vector3d rpy(data[3], data[4], data[5]);  // 单位：弧度！XYZ顺序（Rokae默认）

        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translation() = trans;
        T.linear() = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).matrix();

        Eigen::Matrix4d mat = T.matrix();
        std::array<double, 16> result{};
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(result.data()) = mat;
        return result;
    }

    std::array<double, 16> pose7d_to_matrix(const std::vector<float>& data) {
        if (data.size() < 7) return std::array<double, 16>{};

        Eigen::Vector3d trans(data[0], data[1], data[2]);
        Eigen::Quaterniond quat(data[6], data[3], data[4], data[5]);  // w,x,y,z

        if (std::abs(quat.norm() - 1.0) > 1e-3) {
            quat.normalize();  // 防止数值误差
        }

        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translation() = trans;
        T.linear() = quat.toRotationMatrix();

        Eigen::Matrix4d mat = T.matrix();
        std::array<double, 16> result{};
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(result.data()) = mat;
        return result;
    }

    xMateErProRobot robot_;
    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_sub_;

    std::mutex pose_mutex_;
    std::deque<std::array<double, 16>> pose_queue_;
    std::thread control_thread_;

    bool control_started_ = false;
    bool loop_started_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianTrackerNode>());
    rclcpp::shutdown();
    return 0;
}