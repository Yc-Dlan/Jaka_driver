#include "rclcpp/rclcpp.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/display_robot_state.hpp"  
#include "moveit_msgs/msg/display_trajectory.hpp"   
#include "moveit_msgs/msg/attached_collision_object.hpp"  
#include "moveit_msgs/msg/collision_object.hpp"  
#include "moveit_visual_tools/moveit_visual_tools.h"

#include <moveit_msgs/msg/joint_limits.hpp>
#include <moveit/robot_state/robot_state.h>

#include "std_srvs/srv/empty.hpp"
// ====== 新增1：添加摇杆订阅需要的头文件 ======
#include "std_msgs/msg/float64_multi_array.hpp"
#include <mutex>
#include <atomic>
// ==============================================

using namespace std;

// ====== 新增2：全局变量-摇杆指令相关（线程安全） ======
std::vector<double> target_joint_values(6, 0.0);  // 存储摇杆发布的6个关节值
std::mutex joint_mutex;                           // 互斥锁-保证线程安全
std::atomic<bool> has_new_target{false};          // 原子变量-标记是否有新的摇杆指令
// JAKA Zu3 官方关节限位（绝对安全，和你的Python映射匹配）
const std::vector<double> JOINT_MIN = {0.0,  0.0,  -3.14, 0.0,  0.0,  0.0};
const std::vector<double> JOINT_MAX = {3.14, 3.14,  0.0,  3.14, 3.14, 3.14};
// ====================================================

void sigintHandler(int /*sig*/) {
    rclcpp::shutdown();
}

// ====== 新增3：摇杆指令话题回调函数（核心！接收Python发布的关节值） ======
void jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr joint_msg)
{
    std::lock_guard<std::mutex> lock(joint_mutex);
    // 数据校验：必须是6个关节值
    if(joint_msg->data.size() != 6)
    {
        RCLCPP_WARN(rclcpp::get_logger("jaka_planner"), "摇杆指令错误：关节值数量=%zu，必须为6个！", joint_msg->data.size());
        return;
    }
    // 关节限位保护：将摇杆指令限制在官方安全范围内
    for(int i=0; i<6; i++)
    {
        target_joint_values[i] = std::max(JOINT_MIN[i], std::min(JOINT_MAX[i], joint_msg->data[i]));
    }
    // 标记有新指令
    has_new_target = true;
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "✅ 收到摇杆关节指令: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                target_joint_values[0], target_joint_values[1], target_joint_values[2],
                target_joint_values[3], target_joint_values[4], target_joint_values[5]);
}
// ==========================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    signal(SIGINT, sigintHandler);
    auto node = rclcpp::Node::make_shared("jaka_planner", options);

    string model = node->declare_parameter<string>("model", "zu3");
    string PLANNING_GROUP = "jaka_" + model;
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Using PLANNING_GROUP: %s", PLANNING_GROUP.c_str());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    thread spinner_thread([&executor]() {
        executor.spin();
    });
    
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    rclcpp::Duration du_1(5, 0);

    // ====== 新增4：创建话题订阅器，订阅Python摇杆节点的指令 ======
    auto joint_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/jaka_target_joints",  // 话题名：和你的Python摇杆代码完全一致，无需修改
        10,                     // 消息队列大小
        jointTargetCallback     // 回调函数：收到指令后执行
    );
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "✅ 已订阅摇杆话题: /jaka_target_joints");
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "✅ 摇杆控制模式已开启，拨动摇杆即可控制机械臂！");
    // =============================================================

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success =(move_group.plan(my_plan)== moveit::core::MoveItErrorCode::SUCCESS);

    // ====== 注释掉官方的「固定轨迹测试代码」（保留代码，方便你后续测试） ======
    // 原官方固定轨迹代码全部保留，用注释包裹，不执行，你想恢复的话删掉注释即可
    /*
    vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    for (int i = 0; i < 2; i++)
    {
        move_group.setStartStateToCurrentState();
        joint_group_positions[0] = 0.0;  
        joint_group_positions[1] = 1.57;  
        joint_group_positions[2] = -1.57;  
        joint_group_positions[3] = 1.57;  
        joint_group_positions[4] = 1.57;  
        joint_group_positions[5] = 0.0;  
        move_group.setJointValueTarget(joint_group_positions);
        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Visualizing Plan 1 success: %s", success ? "True" : "False");
        move_group.move();
        rclcpp::sleep_for(chrono::milliseconds(500));

        move_group.setStartStateToCurrentState();
        joint_group_positions[0] = 1.57;  
        joint_group_positions[1] = 1.57;  
        joint_group_positions[2] = -1.57;  
        joint_group_positions[3] = 1.57;  
        joint_group_positions[4] = 1.57;  
        joint_group_positions[5] = 0.0;  
        move_group.setJointValueTarget(joint_group_positions);
        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Visualizing Plan 2 success: %s", success ? "True" : "False");
        move_group.move();
        rclcpp::sleep_for(chrono::milliseconds(500));

        joint_group_positions[0] = 1.57;  
        joint_group_positions[1] = 1.57;  
        joint_group_positions[2] = 1.57;  
        joint_group_positions[3] = 1.57;  
        joint_group_positions[4] = 1.57;  
        joint_group_positions[5] = 0.0;  
        move_group.setJointValueTarget(joint_group_positions);
        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Visualizing Plan 3 success: %s", success ? "True" : "False");
        move_group.move();
        rclcpp::sleep_for(chrono::milliseconds(500));

    }

    move_group.setStartStateToCurrentState();
    joint_group_positions[0] = 0.0;  
    joint_group_positions[1] = 1.57;  
    joint_group_positions[2] = -1.57;  
    joint_group_positions[3] = 1.57;  
    joint_group_positions[4] = 1.57;  
    joint_group_positions[5] = 0.0;  
    move_group.setJointValueTarget(joint_group_positions);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "Visualizing Plan 4 success: %s", success ? "True" : "False");
    move_group.move();
    rclcpp::sleep_for(chrono::seconds(1));
    */

    // ====== 新增5：实时摇杆控制主循环（核心！替代原固定轨迹） ======
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "==================================");
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "进入实时摇杆控制模式，等待指令...");
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "关节限位：JOINT_MIN[0~-3.14], JOINT_MAX[3.14~0]");
    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "==================================");
    rclcpp::Rate rate(20); // 20Hz循环频率，流畅无卡顿
    vector<double> current_joints;
    while(rclcpp::ok())
    {
        // 如果收到摇杆的新指令，执行规划+运动
        if(has_new_target)
        {
            std::lock_guard<std::mutex> lock(joint_mutex);
            // 设置起始位姿为当前位姿
            move_group.setStartStateToCurrentState();
            // 设置摇杆传来的目标关节值
            move_group.setJointValueTarget(target_joint_values);
            // 规划运动轨迹
            success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if(success)
            {
                RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "✅ 规划成功，执行运动！");
                move_group.move(); // 执行运动
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("jaka_planner"), "❌ 规划失败，请微调摇杆位置！");
            }
            // 重置指令标记，等待下一次摇杆操作
            has_new_target = false;
        }
        rate.sleep();
    }
    // =============================================================

    rclcpp::shutdown();
    if (spinner_thread.joinable()) {
        spinner_thread.join();
    }

    return 0;
}
