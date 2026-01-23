/**
 * 文件路径: jaka_planner/src/hybrid_servo_node.cpp
 * 功能: 直连控制 + 单关节无缝切换 + 工具坐标系笛卡尔
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>

using namespace std::chrono_literals;

class HybridServoNode : public rclcpp::Node {
public:
    HybridServoNode() : Node("hybrid_test") {
        last_cart_time_ = this->now();
        last_joint_time_ = this->now();

        // 订阅笛卡尔速度
        cart_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/jaka_cartesian_cmd", 10, 
            [this](const geometry_msgs::msg::Twist::SharedPtr msg){ 
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_twist_ = *msg;
                last_cart_time_ = this->now();
                mode_ = CARTESIAN; 
            });

        // 订阅关节速度 (注意：这里现在接收的是速度！)
        joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/jaka_target_joints", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_joint_vels_ = msg->data; // 存为速度
                last_joint_time_ = this->now();
                mode_ = JOINT_SINGLE;
            });

        // 直连控制器发布者
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/jaka_zu20_controller/joint_trajectory", 10);

        // MoveIt 初始化
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this), "jaka_zu20");
        
        robot_state_ = move_group_->getCurrentState();
        joint_model_group_ = move_group_->getRobotModel()->getJointModelGroup("jaka_zu20");
        end_effector_link_ = move_group_->getEndEffectorLink();
        if (end_effector_link_.empty()) end_effector_link_ = joint_model_group_->getLinkModelNames().back();
        joint_names_ = joint_model_group_->getVariableNames();

        // 50Hz 控制循环
        timer_ = this->create_wall_timer(20ms, std::bind(&HybridServoNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "✅ 混合控制器已就绪 (支持单关节/笛卡尔无缝切换)");
    }

private:
    enum ControlMode { IDLE, CARTESIAN, JOINT_SINGLE };
    ControlMode mode_ = IDLE;
    
    std::mutex data_mutex_;
    geometry_msgs::msg::Twist latest_twist_;
    std::vector<double> latest_joint_vels_; // 缓存关节速度
    rclcpp::Time last_cart_time_, last_joint_time_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::string end_effector_link_;
    std::vector<std::string> joint_names_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cart_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void controlLoop() {
        // 超时重置
        auto now = this->now();
        if ((now - last_cart_time_).seconds() > 0.5 && 
            (now - last_joint_time_).seconds() > 0.5) {
            mode_ = IDLE;
            return; 
        }

        // 1. 获取当前真实状态 (这是无缝切换的基石)
        robot_state_ = move_group_->getCurrentState();
        robot_state_->update(); 

        std::vector<double> target_joints;
        robot_state_->copyJointGroupPositions(joint_model_group_, target_joints);
        double dt = 0.02; // 50Hz

        bool has_new_cmd = false;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            if (mode_ == CARTESIAN) {
                // === 笛卡尔模式 (工具坐标系) ===
                Eigen::Vector3d lin_vel_tool(latest_twist_.linear.x, latest_twist_.linear.y, latest_twist_.linear.z);
                Eigen::Vector3d ang_vel_tool(latest_twist_.angular.x, latest_twist_.angular.y, latest_twist_.angular.z);

                if (lin_vel_tool.norm() > 0.001 || ang_vel_tool.norm() > 0.001) {
                    // 工具 -> 基座 变换
                    const Eigen::Isometry3d& transform = robot_state_->getGlobalLinkTransform(end_effector_link_);
                    Eigen::Matrix3d rotation = transform.rotation();
                    Eigen::Vector3d lin_vel_base = rotation * lin_vel_tool;
                    Eigen::Vector3d ang_vel_base = rotation * ang_vel_tool;

                    Eigen::VectorXd twist_base(6);
                    twist_base << lin_vel_base.x(), lin_vel_base.y(), lin_vel_base.z(),
                                  ang_vel_base.x(), ang_vel_base.y(), ang_vel_base.z();

                    // 微分逆解求解目标关节
                    if (robot_state_->setFromDiffIK(joint_model_group_, twist_base, end_effector_link_, dt)) {
                        robot_state_->copyJointGroupPositions(joint_model_group_, target_joints);
                        has_new_cmd = true;
                    }
                }

            } else if (mode_ == JOINT_SINGLE) {
                // === 单关节模式 (增量积分) ===
                // Python发来的是 velocity 数组 (其中只有一个不为0，其他为0)
                if (latest_joint_vels_.size() == target_joints.size()) {
                    for(size_t i=0; i<target_joints.size(); ++i) {
                        // 积分: Target = Current + Velocity * dt
                        // 无论切换到哪个关节，其他关节 Vel=0，所以保持 Current 不变 -> 无突变
                        target_joints[i] += latest_joint_vels_[i] * dt;
                        
                        // 简单的运动阈值判断
                        if (std::abs(latest_joint_vels_[i]) > 0.01) has_new_cmd = true;
                    }
                }
            }
        }

        if (!has_new_cmd) return;

        // 2. 发送直连指令
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.header.stamp = now;
        traj_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_joints;
        point.time_from_start = rclcpp::Duration::from_seconds(0.04); 
        
        traj_msg.points.push_back(point);
        traj_pub_->publish(traj_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<HybridServoNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}