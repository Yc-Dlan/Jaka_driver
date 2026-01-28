#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <mutex>
#include <atomic>
#include <thread>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/robot_state/robot_state.h>
#include <tf2_eigen/tf2_eigen.hpp> 

using namespace std;

// ====== 全局变量 ======
std::vector<double> target_joint_values(6, 0.0);
geometry_msgs::msg::Twist target_cart_twist;
std::mutex data_mutex;
std::atomic<int> control_mode{0}; 

// 放开限位
const std::vector<double> JOINT_MIN = {-6.28, -6.28, -6.28, -6.28, -6.28, -6.28};
const std::vector<double> JOINT_MAX = { 6.28,  6.28,  6.28,  6.28,  6.28,  6.28};

void jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr joint_msg)
{
    if(joint_msg->data.size() != 6) return;
    std::lock_guard<std::mutex> lock(data_mutex);
    for(int i=0; i<6; i++) target_joint_values[i] = joint_msg->data[i];
    control_mode = 0; 
}

void cartesianTargetCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    target_cart_twist = *twist_msg;
    if(abs(twist_msg->linear.x) > 0.0001 || abs(twist_msg->linear.y) > 0.0001 || abs(twist_msg->linear.z) > 0.0001 ||
       abs(twist_msg->angular.x) > 0.0001 || abs(twist_msg->angular.y) > 0.0001 || abs(twist_msg->angular.z) > 0.0001)
    {
        control_mode = 1; 
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    auto node = rclcpp::Node::make_shared("jaka_planner", options);

    rclcpp::QoS qos_profile(1); 
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    auto joint_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/jaka_target_joints", qos_profile, jointTargetCallback);

    auto cart_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "/jaka_cartesian_cmd", qos_profile, cartesianTargetCallback);

    // ⚠️ 确认组名
    std::string PLANNING_GROUP = "jaka_zu20"; 
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    
    // 获取 RobotModel 和 RobotState (用于手动 IK 解算)
    moveit::core::RobotModelConstPtr robot_model = move_group.getRobotModel();
    moveit::core::RobotStatePtr kinematic_state = move_group.getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);

    move_group.setMaxVelocityScalingFactor(1.0);     
    move_group.setMaxAccelerationScalingFactor(1.0); 
    move_group.setPlanningTime(0.05);
    // 强制设置参考坐标系为基座，防止飘飞
    move_group.setPoseReferenceFrame("base_link"); 

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "🔥 防乱飞版控制启动 (Anti-Flip IK)");

    rclcpp::Rate rate(50); 
    
    while(rclcpp::ok())
    {
        int current_mode_local = control_mode.exchange(0);

        // 更新当前的运动学状态
        kinematic_state = move_group.getCurrentState();

        if(current_mode_local == 0) 
        {
            // === 关节模式 (保持不变) ===
            std::vector<double> target_copy;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                target_copy = target_joint_values;
            }
            // 简单防抖
            double sum = 0; for(auto v:target_copy) sum+=abs(v);
            if(sum > 0.01) {
                move_group.setJointValueTarget(target_copy);
                move_group.asyncMove();
            }
        }
        else if (current_mode_local == 1)
        {
            // === 笛卡尔模式 (引入 IK 校验) ===
            geometry_msgs::msg::Twist twist;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                twist = target_cart_twist;
            }

            // 1. 计算目标 Pose
            const Eigen::Isometry3d& current_pose_eigen = kinematic_state->getGlobalLinkTransform("Link_06"); 
            geometry_msgs::msg::Pose current_pose = tf2::toMsg(current_pose_eigen);

            // 2. 矩阵变换 (Local -> Global)
            tf2::Quaternion q_current;
            tf2::fromMsg(current_pose.orientation, q_current);
            tf2::Matrix3x3 mat_rot(q_current);
            
            tf2::Vector3 vec_local(twist.linear.x, twist.linear.y, twist.linear.z);
            tf2::Vector3 vec_global = mat_rot * vec_local;

            // 钳制步长 (2mm)
            double max_step = 0.002;
            if (vec_global.length() > max_step) vec_global = vec_global.normalized() * max_step;

            current_pose.position.x += vec_global.x();
            current_pose.position.y += vec_global.y();
            current_pose.position.z += vec_global.z();

            tf2::Quaternion q_delta;
            q_delta.setRPY(twist.angular.x, twist.angular.y, twist.angular.z);
            tf2::Quaternion q_new = q_current * q_delta;
            q_new.normalize();
            current_pose.orientation = tf2::toMsg(q_new);

            // ==============================================================
            // 🛡️ 核心防乱飞逻辑：手动 IK 解算与校验
            // ==============================================================
            
            // 3. 记录当前的关节角度
            std::vector<double> current_joints;
            kinematic_state->copyJointGroupPositions(joint_model_group, current_joints);

            // 4. 调用 IK 解算器 (寻找离 current_joints 最近的解)
            // timeout 设置为 0.05s
            bool found_ik = kinematic_state->setFromIK(joint_model_group, current_pose, 0.05);

            if (found_ik)
            {
                std::vector<double> new_joints;
                kinematic_state->copyJointGroupPositions(joint_model_group, new_joints);

                // 5. 检查解的连续性 (防止突变)
                bool is_safe = true;
                for(size_t i=0; i<6; i++) {
                    // 如果任何一个关节需要在 20ms 内转动超过 0.5 弧度 (约30度)，说明构型翻转了
                    if(abs(new_joints[i] - current_joints[i]) > 0.5) {
                        is_safe = false;
                        break;
                    }
                }

                if(is_safe) {
                    // 如果解是安全的（离当前位置很近），则执行
                    move_group.setJointValueTarget(new_joints);
                    move_group.asyncMove();
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("jaka_planner"), "⚠️ 拒绝执行：检测到构型翻转 (Joint Jump)");
                }
            }
            else {
                // RCLCPP_WARN(rclcpp::get_logger("jaka_planner"), "无 IK 解");
            }
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}