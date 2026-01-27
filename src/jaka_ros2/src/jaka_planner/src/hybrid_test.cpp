/**
 * 文件路径: jaka_planner/src/hybrid_servo_node.cpp
 * 修复: 解决 RobotModelLoader 构造函数参数类型不匹配的编译错误
 * 采用二段式初始化 (init pattern)
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

using namespace std::chrono_literals;

class HybridServoNode : public rclcpp::Node {
public:
    HybridServoNode() : Node("hybrid_test") {
        last_cart_time_ = this->now();
        last_joint_time_ = this->now();

        // 1. 订阅控制指令
        cart_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/jaka_cartesian_cmd", 10, 
            [this](const geometry_msgs::msg::Twist::SharedPtr msg){ 
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_twist_ = *msg;
                last_cart_time_ = this->now();
                mode_ = CARTESIAN; 
            });

        joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/jaka_target_joints", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_joint_vels_ = msg->data; 
                last_joint_time_ = this->now();
                mode_ = JOINT_SINGLE;
            });

        // 2. 订阅真实关节状态
        state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg){
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_joint_state_ = *msg;
            });

        // 3. 直连控制器发布
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/jaka_zu20_controller/joint_trajectory", 10);
            
        RCLCPP_INFO(this->get_logger(), "⏳ 节点构造完成，等待初始化 MoveIt...");
    }

    // [关键修复] 将 MoveIt 的初始化移到这个单独的函数中
    // 这样我们就可以安全地使用 shared_from_this() 了
    void init() {
        // 4. 加载运动学模型
        // shared_from_this() 只有在对象被 make_shared 管理后才能调用
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
            shared_from_this(), "robot_description");
            
        robot_model_ = robot_model_loader_->getModel();
        joint_model_group_ = robot_model_->getJointModelGroup("jaka_zu20");
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        
        // 获取标准关节顺序 (用于修正乱序问题)
        canonical_joint_names_ = joint_model_group_->getVariableNames();
        
        // 硬编码获取末端
        end_effector_link_ = "Link_06"; 
        
        RCLCPP_INFO(this->get_logger(), "✅ 运动学核心加载成功 | 末端: %s", end_effector_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "🔍 标准关节顺序: %s, %s...", 
            canonical_joint_names_[0].c_str(), canonical_joint_names_[1].c_str());

        // 5. 启动循环 (初始化完成后再启动，防止空指针崩溃)
        timer_ = this->create_wall_timer(20ms, std::bind(&HybridServoNode::controlLoop, this));
    }

private:
    enum ControlMode { IDLE, CARTESIAN, JOINT_SINGLE };
    ControlMode mode_ = IDLE;
    
    std::mutex data_mutex_, state_mutex_;
    geometry_msgs::msg::Twist latest_twist_;
    std::vector<double> latest_joint_vels_;
    sensor_msgs::msg::JointState current_joint_state_;
    rclcpp::Time last_cart_time_, last_joint_time_;

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
        // 安全检查：确保 init() 已被调用
        if (!robot_state_) return;

        auto now = this->now();
        if ((now - last_cart_time_).seconds() > 0.5 && 
            (now - last_joint_time_).seconds() > 0.5) {
            mode_ = IDLE;
            return; 
        }

        // 1. 同步状态 (解决乱序的关键)
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (current_joint_state_.name.empty()) return;
            
            // MoveIt 会自动根据名字匹配值，无视 joint_states 的乱序
            robot_state_->setVariablePositions(current_joint_state_.name, current_joint_state_.position);
            robot_state_->update(); 
        }

        // 获取对齐后的当前位置 (标准顺序)
        std::vector<double> target_joints;
        robot_state_->copyJointGroupPositions(joint_model_group_, target_joints);
        
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
                        robot_state_->copyJointGroupPositions(joint_model_group_, target_joints);
                        has_new_cmd = true;
                    }
                }

            } else if (mode_ == JOINT_SINGLE) {
                // 单关节逻辑
                if (latest_joint_vels_.size() == 6) {
                    for(size_t i=0; i<6; ++i) {
                        if (std::abs(latest_joint_vels_[i]) > 0.001) {
                            // 标准顺序直接相加
                            target_joints[i] += latest_joint_vels_[i] * dt;
                            has_new_cmd = true;
                        }
                    }
                }
            }
        }

        if (has_new_cmd) {
            trajectory_msgs::msg::JointTrajectory traj_msg;
            traj_msg.header.stamp.sec = 0;
            traj_msg.header.stamp.nanosec = 0;
            
            // [关键] 告诉控制器：这组数据是按照标准顺序排列的
            // 只要控制器能读懂名字，它就会自己去匹配
            traj_msg.joint_names = canonical_joint_names_; 

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = target_joints;
            point.time_from_start = rclcpp::Duration::from_seconds(0.1); 
            
            traj_msg.points.push_back(point);
            traj_pub_->publish(traj_msg);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    
    // 1. 创建节点
    auto node = std::make_shared<HybridServoNode>();
    
    // 2. [关键修复] 手动调用初始化，传入 shared_ptr
    node->init();
    
    // 3. 运行
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}