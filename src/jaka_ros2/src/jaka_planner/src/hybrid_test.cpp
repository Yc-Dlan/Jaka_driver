#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>

using namespace std::chrono_literals;

class HybridServoNode : public rclcpp::Node {
public:
    HybridServoNode() : Node("hybrid_test") {
        // 初始化所有时间戳
        last_cart_time_ = this->now();
        last_joint_time_ = this->now();
        last_cmd_time_ = this->now(); 

        // 订阅控制指令
        cart_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/jaka_cartesian_cmd", 10, 
            [this](const geometry_msgs::msg::Twist::SharedPtr msg){ 
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_twist_ = *msg;
                last_cart_time_ = this->now();
                last_cmd_time_ = this->now(); // 激活
                mode_ = CARTESIAN; 
            });

        joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/jaka_target_joints", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_joint_vels_ = msg->data; 
                last_joint_time_ = this->now();
                last_cmd_time_ = this->now(); // 激活
                mode_ = JOINT_SINGLE;
            });

        // 使用 SensorDataQoS (Best Effort) 订阅状态
        state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(), 
            [this](const sensor_msgs::msg::JointState::SharedPtr msg){
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_sensor_state_ = *msg;
                is_sensor_received_ = true; // 标记数据已送达
            });

        // 直连控制器发布
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/jaka_zu20_controller/joint_trajectory", 10);

        RCLCPP_INFO(this->get_logger(), "⏳ 等待 MoveIt 初始化...");
    }

    void init() {
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
            shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();
        joint_model_group_ = robot_model_->getJointModelGroup("jaka_zu20");
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        
        canonical_joint_names_ = joint_model_group_->getVariableNames();
        end_effector_link_ = "Link_06"; 
        
        // 清空内部状态
        internal_target_joints_.clear();
        
        RCLCPP_INFO(this->get_logger(), "✅ 积分平滑控制器已启动 (QoS Fixed)");
        
        timer_ = this->create_wall_timer(20ms, std::bind(&HybridServoNode::controlLoop, this));
    }

private:
    enum ControlMode { IDLE, CARTESIAN, JOINT_SINGLE };
    ControlMode mode_ = IDLE;
    
    std::mutex data_mutex_, state_mutex_;
    geometry_msgs::msg::Twist latest_twist_;
    std::vector<double> latest_joint_vels_;
    sensor_msgs::msg::JointState current_sensor_state_;
    bool is_sensor_received_ = false;
    
    // 内部积分状态 (平滑的核心)
    std::vector<double> internal_target_joints_;
    bool need_resync_ = true; 
    rclcpp::Time last_cmd_time_, last_cart_time_, last_joint_time_;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::string end_effector_link_;
    std::vector<std::string> canonical_joint_names_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cart_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void controlLoop() {
        if (!robot_state_) return;

        // 1. 活跃状态检查 (0.5s 超时)
        bool is_active = (this->now() - last_cmd_time_).seconds() < 0.5;

        if (!is_active) {
            if (mode_ != IDLE) {
                need_resync_ = true; // 手柄松开，下次必须重置
                mode_ = IDLE;
                RCLCPP_INFO(this->get_logger(), "⏸️ 待机中... (松开手柄)");
            }
            return;
        }

        // 2. 同步逻辑 (Anti-Jump)
        // 只在"刚按下手柄"的那一瞬间读取传感器
        if (need_resync_) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!is_sensor_received_ || current_sensor_state_.name.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "⚠️ 等待传感器数据同步...");
                return; 
            }
            
            // MoveIt 自动对齐乱序数据
            robot_state_->setVariablePositions(current_sensor_state_.name, current_sensor_state_.position);
            // 存入积分器，作为数学计算的起点
            robot_state_->copyJointGroupPositions(joint_model_group_, internal_target_joints_);
            
            need_resync_ = false;
            RCLCPP_INFO(this->get_logger(), "🔄 状态已同步，开始纯积分控制");
        }

        // 安全检查：防止积分器未初始化导致崩溃
        if (internal_target_joints_.size() < 6) return;

        // 3. 计算逻辑 (完全基于 internal_target_joints_)
        // 这一步完全不读取 Sensor，所以绝对没有噪声
        robot_state_->setJointGroupPositions(joint_model_group_, internal_target_joints_);
        robot_state_->update(); // 更新FK

        double dt = 0.02; 
        bool has_new_cmd = false;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            if (mode_ == CARTESIAN) {
                Eigen::Vector3d lin_vel_tool(latest_twist_.linear.x, latest_twist_.linear.y, latest_twist_.linear.z);
                Eigen::Vector3d ang_vel_tool(latest_twist_.angular.x, latest_twist_.angular.y, latest_twist_.angular.z);

                if (lin_vel_tool.norm() > 0.001 || ang_vel_tool.norm() > 0.001) {
                    const Eigen::Isometry3d& transform = robot_state_->getGlobalLinkTransform(end_effector_link_);
                    Eigen::Matrix3d rotation = transform.rotation();
                    Eigen::VectorXd twist_base(6);
                    twist_base.head(3) = rotation * lin_vel_tool;
                    twist_base.tail(3) = rotation * ang_vel_tool;

                    if (robot_state_->setFromDiffIK(joint_model_group_, twist_base, end_effector_link_, dt)) {
                        // 将无噪声的计算结果存回积分器
                        robot_state_->copyJointGroupPositions(joint_model_group_, internal_target_joints_);
                        has_new_cmd = true;
                    }
                }

            } else if (mode_ == JOINT_SINGLE) {
                if (latest_joint_vels_.size() == 6) {
                    for(size_t i=0; i<6; ++i) {
                        if (std::abs(latest_joint_vels_[i]) > 0.001) {
                            // 直接在积分器上累加
                            internal_target_joints_[i] += latest_joint_vels_[i] * dt;
                            has_new_cmd = true;
                        }
                    }
                }
            }
        }

        // 4. 发送指令
        if (has_new_cmd) {
            trajectory_msgs::msg::JointTrajectory traj_msg;
            traj_msg.header.stamp.sec = 0;
            traj_msg.header.stamp.nanosec = 0;
            traj_msg.joint_names = canonical_joint_names_;

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = internal_target_joints_; // 发送纯净值
            point.time_from_start = rclcpp::Duration::from_seconds(0.1); 
            
            traj_msg.points.push_back(point);
            traj_pub_->publish(traj_msg);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<HybridServoNode>();
    node->init(); 
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}