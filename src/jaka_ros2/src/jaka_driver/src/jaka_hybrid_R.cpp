#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// 引入 JAKA 官方服务类型
#include <jaka_msgs/srv/servo_move_enable.hpp>
#include <jaka_msgs/srv/servo_move.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <atomic>
#include <algorithm>

using namespace std::chrono_literals;

class HybridServoRealNode : public rclcpp::Node {
public:
    HybridServoRealNode() : Node("hybrid_servo_real"), mode_(IDLE) {
        
        last_cmd_time_ = this->now(); 
        current_rpy_.resize(3, 0.0);
        latest_joint_vels_.resize(6, 0.0);

        // 1. 订阅前端 Python 发来的指令 (速度指令)
        cart_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/jaka_cartesian_cmd", 10, 
            [this](const geometry_msgs::msg::Twist::SharedPtr msg){ 
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_twist_ = *msg;
                // 切入笛卡尔模式时，强制清零关节指令缓存
                std::fill(latest_joint_vels_.begin(), latest_joint_vels_.end(), 0.0);
                last_cmd_time_ = this->now(); 
                mode_ = CARTESIAN; 
            });

        joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/jaka_target_joints", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
                std::lock_guard<std::mutex> lock(data_mutex_);
                if(msg->data.size() == 6) latest_joint_vels_ = msg->data; 
                // 切入关节模式时，强制清零笛卡尔指令缓存
                latest_twist_ = geometry_msgs::msg::Twist(); 
                last_cmd_time_ = this->now(); 
                mode_ = JOINT_SINGLE;
            });

        // 2. 订阅真实机械臂的末端姿态 (用于笛卡尔模式的 Local -> Global 转换)
        // 注意：JAKA 官方驱动发布的 tool_position 中，角度是角度制 (Degree)
        tool_pos_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/jaka_driver/tool_position", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg){
                std::lock_guard<std::mutex> lock(pose_mutex_);
                current_rpy_[0] = msg->twist.angular.x * M_PI / 180.0;
                current_rpy_[1] = msg->twist.angular.y * M_PI / 180.0;
                current_rpy_[2] = msg->twist.angular.z * M_PI / 180.0;
            });

        // 3. 创建 JAKA 底层伺服客户端
        servo_enable_client_ = this->create_client<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/servo_move_enable");
        servo_p_client_ = this->create_client<jaka_msgs::srv::ServoMove>("/jaka_driver/servo_p");
        servo_j_client_ = this->create_client<jaka_msgs::srv::ServoMove>("/jaka_driver/servo_j");

        RCLCPP_INFO(this->get_logger(), "⏳ 正在等待 JAKA 真实控制柜服务上线...");
        while (!servo_enable_client_->wait_for_service(1s) || !servo_p_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
        }

        // 4. 异步请求开启伺服模式
        auto req = std::make_shared<jaka_msgs::srv::ServoMoveEnable::Request>();
        req->enable = true;
        servo_enable_client_->async_send_request(req, [this](rclcpp::Client<jaka_msgs::srv::ServoMoveEnable>::SharedFuture future) {
            if (future.get()->ret == 1) {
                RCLCPP_INFO(this->get_logger(), "✅ JAKA 硬件伺服模式已开启！可以下发摇杆指令。");
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 开启伺服失败！");
            }
        });

        // 5. 开启 125Hz 控制循环
        timer_ = this->create_wall_timer(8ms, std::bind(&HybridServoRealNode::controlLoop, this));
    }

private:
    enum ControlMode { IDLE, CARTESIAN, JOINT_SINGLE };
    std::atomic<ControlMode> mode_;
    
    std::mutex data_mutex_, pose_mutex_;
    geometry_msgs::msg::Twist latest_twist_;
    std::vector<double> latest_joint_vels_;
    std::vector<double> current_rpy_;
    
    rclcpp::Time last_cmd_time_;
    const double dt_ = 0.008; // 125Hz(8ms)

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cart_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr tool_pos_sub_;
    
    rclcpp::Client<jaka_msgs::srv::ServoMoveEnable>::SharedPtr servo_enable_client_;
    rclcpp::Client<jaka_msgs::srv::ServoMove>::SharedPtr servo_p_client_;
    rclcpp::Client<jaka_msgs::srv::ServoMove>::SharedPtr servo_j_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void controlLoop() {
        if ((this->now() - last_cmd_time_).seconds() > 0.1) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (mode_ != IDLE) {
                mode_ = IDLE;
                // 超时后必须强行把内部速度清零
                latest_twist_ = geometry_msgs::msg::Twist();
                std::fill(latest_joint_vels_.begin(), latest_joint_vels_.end(), 0.0);
                RCLCPP_INFO(this->get_logger(), "⏸️ 安全键已松开(或无指令)，指令强制清零，停止下发...");
            }
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (mode_ == CARTESIAN) 
        {
            // === 笛卡尔增量模式 ===
            auto req = std::make_shared<jaka_msgs::srv::ServoMove::Request>();

            // 1. 计算线速度增量 (Tool 坐标系下)
            tf2::Vector3 vec_local(latest_twist_.linear.x * dt_, 
                                   latest_twist_.linear.y * dt_, 
                                   latest_twist_.linear.z * dt_);

            // 2. 利用底层反馈的 RPY 将增量转换到 Base 坐标系
            tf2::Quaternion q_current;
            {
                std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                q_current.setRPY(current_rpy_[0], current_rpy_[1], current_rpy_[2]);
            }
            tf2::Matrix3x3 mat_rot(q_current);
            tf2::Vector3 vec_global = mat_rot * vec_local;

            // 3. 【极其重要】JAKA 底层伺服接口要求距离单位为 毫米(mm)！
            double dx_mm = vec_global.x() * 1000.0;
            double dy_mm = vec_global.y() * 1000.0;
            double dz_mm = vec_global.z() * 1000.0;

            // 钳制单步最大位移，防止摇杆突变导致机械臂抽搐 (最大约 2mm/step)
            dx_mm = std::clamp(dx_mm, -2.0, 2.0);
            dy_mm = std::clamp(dy_mm, -2.0, 2.0);
            dz_mm = std::clamp(dz_mm, -2.0, 2.0);

            req->pose.push_back(dx_mm);
            req->pose.push_back(dy_mm);
            req->pose.push_back(dz_mm);

            // 4. 角度增量 (直接传递，单位为 rad)
            req->pose.push_back(latest_twist_.angular.x * dt_);
            req->pose.push_back(latest_twist_.angular.y * dt_);
            req->pose.push_back(latest_twist_.angular.z * dt_);

            // 只有当有实际位移时才下发指令 (避免频繁发送纯0指令干扰底层伺服池)
            if (std::abs(dx_mm) > 0.001 || std::abs(dy_mm) > 0.001 || std::abs(dz_mm) > 0.001 ||
                std::abs(latest_twist_.angular.x) > 0.001 || std::abs(latest_twist_.angular.y) > 0.001 || std::abs(latest_twist_.angular.z) > 0.001) {
                servo_p_client_->async_send_request(req);
            }
        }
        else if (mode_ == JOINT_SINGLE) 
        {
            // === 关节增量模式 ===
            auto req = std::make_shared<jaka_msgs::srv::ServoMove::Request>();
            bool has_movement = false;

            for (size_t i = 0; i < 6; ++i) {
                double delta_rad = latest_joint_vels_[i] * dt_;
                // 钳制最大关节速度 (约 0.01 rad/step)
                delta_rad = std::clamp(delta_rad, -0.01, 0.01);
                req->pose.push_back(delta_rad);
                
                if (std::abs(delta_rad) > 0.0001) has_movement = true;
            }

            // 发送关节微增量给控制柜
            if (has_movement) {
                servo_j_client_->async_send_request(req);
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HybridServoRealNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}