#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <mutex>
#include <atomic>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

// 全局变量
std::vector<double> target_joint_values(6, 0.0);
geometry_msgs::msg::Twist target_cart_twist; // 存储笛卡尔增量
std::mutex data_mutex;
std::atomic<int> control_mode{0}; // 0: 无/关节控制, 1: 笛卡尔控制

const std::vector<double> JOINT_MIN = {-6.28, -6.28, -6.28, -6.28, -6.28, -6.28};
const std::vector<double> JOINT_MAX = { 6.28,  6.28,  6.28,  6.28,  6.28,  6.28};

// 关节回调
void jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr joint_msg)
{
    if(joint_msg->data.size() != 6) return;
    std::lock_guard<std::mutex> lock(data_mutex);
    for(int i=0; i<6; i++) {
        target_joint_values[i] = std::max(JOINT_MIN[i], std::min(JOINT_MAX[i], joint_msg->data[i]));
    }
    control_mode = 0; // 标记为关节模式
}

// 笛卡尔回调 (Twist)
void cartesianTargetCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    target_cart_twist = *twist_msg;
    // 只有当有有效输入时，才激活笛卡尔模式
    if(abs(twist_msg->linear.x) > 0.001 || abs(twist_msg->linear.y) > 0.001 || abs(twist_msg->linear.z) > 0.001 ||
       abs(twist_msg->angular.x) > 0.001 || abs(twist_msg->angular.y) > 0.001 || abs(twist_msg->angular.z) > 0.001)
    {
        control_mode = 1; // 标记为笛卡尔模式
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

    // 订阅 1: 关节控制
    auto joint_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/jaka_target_joints", qos_profile, jointTargetCallback);

    // 订阅 2: 笛卡尔控制 (新)
    auto cart_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "/jaka_cartesian_cmd", qos_profile, cartesianTargetCallback);

    std::string PLANNING_GROUP = "jaka_zu20"; // ⚠️ 请确认组名
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    move_group.setMaxVelocityScalingFactor(1.0);     
    move_group.setMaxAccelerationScalingFactor(1.0); 
    // 笛卡尔运动需要更短的规划时间
    move_group.setPlanningTime(0.05); 

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "🔥 全能模式启动 (Joint + Cartesian)");

    rclcpp::Rate rate(50); 
    
    while(rclcpp::ok())
    {
        int current_mode_local = control_mode.exchange(0); // 读取并重置为0，避免重复执行

        if(current_mode_local == 0 && target_joint_values.size() == 6) 
        {
            // === 关节模式处理 ===
            // (注意：这里需要判断 target_joint_values 是否被初始化过，简单起见假设初始非全0或已校准)
            // 实际工程中最好加个 flag has_new_joint_target
            // 为了简化代码逻辑，这里假设 Python 一直在发最新的
            std::vector<double> target_copy;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                target_copy = target_joint_values;
            }
            // 简单判断一下是否全是0 (防止刚启动归零)
            double sum = 0; for(auto v:target_copy) sum+=abs(v);
            if(sum > 0.01) {
                move_group.setJointValueTarget(target_copy);
                move_group.asyncMove();
            }
        }
        else if (current_mode_local == 1)
        {
            // === 笛卡尔模式处理 ===
            geometry_msgs::msg::Twist twist;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                twist = target_cart_twist;
            }

            // 1. 获取当前位姿
            geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
            
            // 2. 叠加线性增量 (简单的 Euler 积分)
            // 注意：这里是在"基坐标系"下移动。如果想在"工具坐标系"移动，需要矩阵变换 (略微复杂)
            current_pose.pose.position.x += twist.linear.x;
            current_pose.pose.position.y += twist.linear.y;
            current_pose.pose.position.z += twist.linear.z;

            // 3. 叠加旋转增量 (使用四元数乘法)
            tf2::Quaternion q_orig, q_rot, q_new;
            tf2::fromMsg(current_pose.pose.orientation, q_orig);
            
            // 将角速度转换为微小旋转四元数 (RPY)
            q_rot.setRPY(twist.angular.x, twist.angular.y, twist.angular.z);
            
            q_new = q_orig * q_rot; // 局部旋转
            // q_new = q_rot * q_orig; // 全局旋转 (根据需求选)
            q_new.normalize();
            current_pose.pose.orientation = tf2::toMsg(q_new);

            // 4. 执行
            move_group.setPoseTarget(current_pose);
            // 笛卡尔规划容易失败(IK解算失败)，asyncMove如果不成功通常会打印警告
            move_group.asyncMove();
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}