#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// 引入 JAKA 官方服务类型
#include <jaka_msgs/srv/servo_move_enable.hpp>

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

        tool_pos_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/jaka_driver/tool_position", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg){
                std::lock_guard<std::mutex> lock(pose_mutex_);
                current_rpy_[0] = msg->twist.angular.x * M_PI / 180.0;
                current_rpy_[1] = msg->twist.angular.y * M_PI / 180.0;
                current_rpy_[2] = msg->twist.angular.z * M_PI / 180.0;
            });

        // 启用 SensorDataQoS
        servo_p_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/jaka_driver/servo_p_cmd", rclcpp::SensorDataQoS());
        servo_j_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/jaka_driver/servo_j_cmd", rclcpp::SensorDataQoS());

        servo_enable_client_ = this->create_client<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/servo_move_enable");

        RCLCPP_INFO(this->get_logger(), "⏳ 正在等待 JAKA 真实控制柜服务上线...");
        while (!servo_enable_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
        }

        auto req = std::make_shared<jaka_msgs::srv::ServoMoveEnable::Request>();
        req->enable = true;
        servo_enable_client_->async_send_request(req, [this](rclcpp::Client<jaka_msgs::srv::ServoMoveEnable>::SharedFuture future) {
            if (future.get()->ret == 1) {
                RCLCPP_INFO(this->get_logger(), "✅ JAKA 硬件伺服模式已开启！可以下发摇杆指令。");
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 开启伺服失败！");
            }
        });

        // 125Hz 控制循环
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
    const double dt_ = 0.008; 

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cart_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr tool_pos_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_p_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_j_pub_;
    rclcpp::Client<jaka_msgs::srv::ServoMoveEnable>::SharedPtr servo_enable_client_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    void controlLoop() {
        if ((this->now() - last_cmd_time_).seconds() > 0.1) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (mode_ != IDLE) {
                mode_ = IDLE;
                latest_twist_ = geometry_msgs::msg::Twist();
                std::fill(latest_joint_vels_.begin(), latest_joint_vels_.end(), 0.0);
                RCLCPP_INFO(this->get_logger(), "⏸️ 安全键已松开，已强制清零指令并刹停...");
            }
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (mode_ == CARTESIAN) 
        {
            tf2::Vector3 vec_local(latest_twist_.linear.x * dt_, 
                                   latest_twist_.linear.y * dt_, 
                                   latest_twist_.linear.z * dt_);

            tf2::Quaternion q_current;
            {
                std::lock_guard<std::mutex> pose_lock(pose_mutex_);
                q_current.setRPY(current_rpy_[0], current_rpy_[1], current_rpy_[2]);
            }
            tf2::Matrix3x3 mat_rot(q_current);
            tf2::Vector3 vec_global = mat_rot * vec_local;
            
            double dx_mm = std::clamp(vec_global.x() * 1000.0, -3.0, 3.0);
            double dy_mm = std::clamp(vec_global.y() * 1000.0, -3.0, 3.0);
            double dz_mm = std::clamp(vec_global.z() * 1000.0, -3.0, 3.0);

            // 四元数姿态积分
            tf2::Vector3 w_local(latest_twist_.angular.x, latest_twist_.angular.y, latest_twist_.angular.z);
            double angle = w_local.length() * dt_; 
            
            tf2::Quaternion q_delta_local;
            if (angle > 1e-6) {
                q_delta_local.setRotation(w_local.normalized(), angle);
            } else {
                q_delta_local.setValue(0, 0, 0, 1);
            }

            tf2::Quaternion q_target = q_current * q_delta_local;
            q_target.normalize();

            double target_roll, target_pitch, target_yaw;
            tf2::Matrix3x3(q_target).getRPY(target_roll, target_pitch, target_yaw);

            double d_roll = target_roll - current_rpy_[0];
            double d_pitch = target_pitch - current_rpy_[1];
            double d_yaw = target_yaw - current_rpy_[2];

            auto normalize_angle = [](double a) {
                while (a > M_PI) a -= 2.0 * M_PI;
                while (a < -M_PI) a += 2.0 * M_PI;
                return a;
            };
            d_roll = std::clamp(normalize_angle(d_roll), -0.01, 0.01);
            d_pitch = std::clamp(normalize_angle(d_pitch), -0.01, 0.01);
            d_yaw = std::clamp(normalize_angle(d_yaw), -0.01, 0.01);

            // [核心修改] 过滤全0死区后，打包为 Float64MultiArray 进行 Topic 发布
            if (std::abs(dx_mm) > 0.001 || std::abs(dy_mm) > 0.001 || std::abs(dz_mm) > 0.001 || angle > 1e-5) {
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = {dx_mm, dy_mm, dz_mm, d_roll, d_pitch, d_yaw};
                servo_p_pub_->publish(cmd_msg);
            }
        }
        else if (mode_ == JOINT_SINGLE) 
        {
            bool has_movement = false;
            std_msgs::msg::Float64MultiArray cmd_msg;

            for (size_t i = 0; i < 6; ++i) {
                double delta_rad = std::clamp(latest_joint_vels_[i] * dt_, -0.01, 0.01);
                cmd_msg.data.push_back(delta_rad);
                if (std::abs(delta_rad) > 0.0001) has_movement = true;
            }

            // [核心修改] 过滤死区后进行 Topic 发布
            if (has_movement) {
                servo_j_pub_->publish(cmd_msg);
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