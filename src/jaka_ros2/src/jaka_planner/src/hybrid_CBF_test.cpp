/**
 * 文件路径: jaka_planner/src/hybrid_test.cpp
 * 功能: 混合控制 (集成 CBF 模块)
 */

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

#include "jaka_planner/CBF_filter.hpp" 

using namespace std::chrono_literals;

class HybridServoNode : public rclcpp::Node {
public:
    HybridServoNode() : Node("hybrid_test") {
        // 初始化时间戳
        last_cart_time_ = this->now();
        last_cmd_time_ = this->now(); 

        cart_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/jaka_cartesian_cmd", 10, 
            [this](const geometry_msgs::msg::Twist::SharedPtr msg){ 
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_twist_ = *msg;
                last_cmd_time_ = this->now();
                mode_ = CARTESIAN; 
            });

        joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/jaka_target_joints", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_joint_vels_ = msg->data; 
                last_cmd_time_ = this->now();
                mode_ = JOINT_SINGLE;
            });

        state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(), 
            [this](const sensor_msgs::msg::JointState::SharedPtr msg){
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_sensor_state_ = *msg;
                is_sensor_received_ = true;
            });

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
        
        internal_target_joints_.clear();

        cbf_filter_.init(shared_from_this());
        
        RCLCPP_INFO(this->get_logger(), "✅ 控制器已启动");
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
    std::vector<double> internal_target_joints_;
    bool need_resync_ = true; 
    rclcpp::Time last_cmd_time_, last_cart_time_;
    
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

    CBFFilter cbf_filter_;

    void controlLoop() {
        if (!robot_state_) return;

        bool is_active = (this->now() - last_cmd_time_).seconds() < 0.5;
        if (!is_active) {
            if (mode_ != IDLE) { need_resync_ = true; mode_ = IDLE; }
        }

        // 同步逻辑
        if (need_resync_) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!is_sensor_received_) return; 
            robot_state_->setVariablePositions(current_sensor_state_.name, current_sensor_state_.position);
            robot_state_->copyJointGroupPositions(joint_model_group_, internal_target_joints_);
            need_resync_ = false;
        }

        if (internal_target_joints_.size() < 6) return;

        // 更新状态，供 CBF 计算使用
        robot_state_->setJointGroupPositions(joint_model_group_, internal_target_joints_);
        robot_state_->update();

        double dt = 0.02; 
        Eigen::VectorXd q_dot_cmd = Eigen::VectorXd::Zero(6); 
        bool has_cmd = false;

        // 仅当激活时处理指令
        if (is_active) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            if (mode_ == CARTESIAN) {
                Eigen::Vector3d lin(latest_twist_.linear.x, latest_twist_.linear.y, latest_twist_.linear.z);
                Eigen::Vector3d ang(latest_twist_.angular.x, latest_twist_.angular.y, latest_twist_.angular.z);
                
                if (lin.norm() > 0.001 || ang.norm() > 0.001) {
                    Eigen::MatrixXd J;
                    robot_state_->getJacobian(joint_model_group_, robot_state_->getLinkModel(end_effector_link_), Eigen::Vector3d::Zero(), J);
                    const Eigen::Isometry3d& tf = robot_state_->getGlobalLinkTransform(end_effector_link_);
                    Eigen::VectorXd v_cart(6);
                    v_cart.head(3) = tf.rotation() * lin;
                    v_cart.tail(3) = tf.rotation() * ang;
                    Eigen::MatrixXd J_t = J.transpose();
                    q_dot_cmd = J_t * (J * J_t + 0.001 * Eigen::MatrixXd::Identity(6, 6)).inverse() * v_cart;
                    has_cmd = true;
                }
            } else if (mode_ == JOINT_SINGLE) {
                if (latest_joint_vels_.size() == 6) {
                    for(size_t i=0; i<6; ++i) q_dot_cmd[i] = latest_joint_vels_[i];
                    has_cmd = true;
                }
            }
        }

        // 🌟 无论是否运动，都调用 filter，确保红球一直显示在 Rviz 里
        cbf_filter_.filter(robot_state_, joint_model_group_, q_dot_cmd);

        if (has_cmd) {
            // 积分
            for (int i=0; i<6; ++i) {
                internal_target_joints_[i] += q_dot_cmd[i] * dt;
            }

            trajectory_msgs::msg::JointTrajectory traj_msg;
            traj_msg.header.stamp = this->now();
            traj_msg.joint_names = canonical_joint_names_;
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = internal_target_joints_; 
            point.time_from_start = rclcpp::Duration::from_seconds(dt); 
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