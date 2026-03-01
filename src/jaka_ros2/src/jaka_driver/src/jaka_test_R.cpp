#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "jaka_msgs/srv/servo_move_enable.hpp"
#include "jaka_msgs/srv/servo_move.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <atomic>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class JakaServoBackend : public rclcpp::Node
{
public:
    JakaServoBackend() : Node("jaka_servo_backend"), control_mode_(0)
    {
        // 1. 初始化数据容器
        target_joint_values_.resize(6, 0.0);
        current_joint_values_.resize(6, 0.0);
        current_rpy_.resize(3, 0.0);

        // 2. 创建 QoS
        rclcpp::QoS qos_profile(1);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // 3. 订阅前端 Python 发来的指令
        joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/jaka_target_joints", qos_profile, std::bind(&JakaServoBackend::joint_cmd_callback, this, std::placeholders::_1));

        cart_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/jaka_cartesian_cmd", qos_profile, std::bind(&JakaServoBackend::cart_cmd_callback, this, std::placeholders::_1));

        // 4. 订阅 JAKA 底层真实状态反馈
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JakaServoBackend::joint_state_callback, this, std::placeholders::_1));

        // JAKA 底层发布的工具位姿 (用于 Tool 到 Base 的坐标系转换)
        tool_pos_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/jaka_driver/tool_position", 10, std::bind(&JakaServoBackend::tool_position_callback, this, std::placeholders::_1));

        // 5. 创建 JAKA 伺服控制客户端
        servo_enable_client_ = this->create_client<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/servo_move_enable");
        servo_p_client_ = this->create_client<jaka_msgs::srv::ServoMove>("/jaka_driver/servo_p");
        servo_j_client_ = this->create_client<jaka_msgs::srv::ServoMove>("/jaka_driver/servo_j");

        // 阻塞等待服务
        while (!servo_enable_client_->wait_for_service(1s) || !servo_p_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "等待 JAKA 伺服底层服务...");
        }

        // 异步开启伺服模式
        auto req = std::make_shared<jaka_msgs::srv::ServoMoveEnable::Request>();
        req->enable = true;
        servo_enable_client_->async_send_request(req, [this](rclcpp::Client<jaka_msgs::srv::ServoMoveEnable>::SharedFuture future) {
            if (future.get()->ret == 1) {
                RCLCPP_INFO(this->get_logger(), "🔥 JAKA 硬件伺服模式已开启！可以下发摇杆指令。");
            }
        });

        // 6. 开启 50Hz 高频控制循环
        timer_ = this->create_wall_timer(20ms, std::bind(&JakaServoBackend::control_loop, this));
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for(int i=0; i<6 && i<(int)msg->position.size(); i++) {
            current_joint_values_[i] = msg->position[i];
        }
    }

    void tool_position_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // 注意：根据 jaka_driver.cpp 源码，它发出来的是角度 (Degree)，需要转成弧度 (Radian)
        current_rpy_[0] = msg->twist.angular.x * M_PI / 180.0;
        current_rpy_[1] = msg->twist.angular.y * M_PI / 180.0;
        current_rpy_[2] = msg->twist.angular.z * M_PI / 180.0;
    }

    void joint_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if(msg->data.size() != 6) return;
        std::lock_guard<std::mutex> lock(data_mutex_);
        for(int i=0; i<6; i++) target_joint_values_[i] = msg->data[i];
        control_mode_ = 1; // 关节模式
        last_cmd_time_ = this->now();
    }

    void cart_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_cart_twist_ = *msg;
        control_mode_ = 2; // 笛卡尔模式
        last_cmd_time_ = this->now();
    }

    void control_loop()
    {
        // 安全保护：如果 100ms 没收到摇杆指令，说明摇杆松开或断线，立即停止发送
        if ((this->now() - last_cmd_time_).seconds() > 0.1) {
            control_mode_ = 0;
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (control_mode_ == 1) // 关节伺服模式
        {
            auto req = std::make_shared<jaka_msgs::srv::ServoMove::Request>();
            for (int i = 0; i < 6; i++) {
                // 计算增量 (目标位置 - 当前位置)
                double delta = target_joint_values_[i] - current_joint_values_[i];
                // 限制单步最大增量，防止抖动撕裂 (限制在约 30度/秒)
                delta = std::clamp(delta, -0.01, 0.01); 
                req->pose.push_back(delta);
            }
            servo_j_client_->async_send_request(req);
        }
        else if (control_mode_ == 2) // 笛卡尔伺服模式
        {
            auto req = std::make_shared<jaka_msgs::srv::ServoMove::Request>();

            // 1. 获取当前工具姿态的旋转矩阵
            tf2::Quaternion q_current;
            q_current.setRPY(current_rpy_[0], current_rpy_[1], current_rpy_[2]);
            tf2::Matrix3x3 mat_rot(q_current);

            // 2. 将摇杆指令 (Tool 系) 转换为基座增量 (Base 系)
            tf2::Vector3 vec_local(target_cart_twist_.linear.x, target_cart_twist_.linear.y, target_cart_twist_.linear.z);
            tf2::Vector3 vec_global = mat_rot * vec_local;

            // 3. 填入 servo_p 增量请求 [X, Y, Z, Rx, Ry, Rz]
            req->pose.push_back(vec_global.x());
            req->pose.push_back(vec_global.y());
            req->pose.push_back(vec_global.z());
            
            // 姿态角增量 (暂存直接映射)
            req->pose.push_back(target_cart_twist_.angular.x);
            req->pose.push_back(target_cart_twist_.angular.y);
            req->pose.push_back(target_cart_twist_.angular.z);

            servo_p_client_->async_send_request(req);
        }
    }

    // 成员变量
    std::vector<double> target_joint_values_;
    std::vector<double> current_joint_values_;
    std::vector<double> current_rpy_;
    geometry_msgs::msg::Twist target_cart_twist_;
    rclcpp::Time last_cmd_time_;
    
    std::atomic<int> control_mode_; // 0=Idle, 1=Joint, 2=Cartesian
    std::mutex data_mutex_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cart_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr tool_pos_sub_;

    rclcpp::Client<jaka_msgs::srv::ServoMoveEnable>::SharedPtr servo_enable_client_;
    rclcpp::Client<jaka_msgs::srv::ServoMove>::SharedPtr servo_p_client_;
    rclcpp::Client<jaka_msgs::srv::ServoMove>::SharedPtr servo_j_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JakaServoBackend>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}