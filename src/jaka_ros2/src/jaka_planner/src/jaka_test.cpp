#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <mutex>
#include <atomic>
#include <thread>
#include <vector>

using namespace std;

// ====== 全局变量 ======
std::vector<double> target_joint_values(6, 0.0);
std::mutex joint_mutex;
std::atomic<bool> has_new_target{false};

// ✅ 核心修正 1：彻底放开限位至 ±360度 (±6.28 rad)
// 防止出现"往一边能动，往另一边动不了"的情况
const std::vector<double> JOINT_MIN = {-6.28, -6.28, -6.28, -6.28, -6.28, -6.28};
const std::vector<double> JOINT_MAX = { 6.28,  6.28,  6.28,  6.28,  6.28,  6.28};

// 话题回调：接收 Python 发来的目标点
void jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr joint_msg)
{
    if(joint_msg->data.size() != 6) return;

    std::lock_guard<std::mutex> lock(joint_mutex);
    for(int i=0; i<6; i++)
    {
        // 简单的限位保护
        target_joint_values[i] = std::max(JOINT_MIN[i], std::min(JOINT_MAX[i], joint_msg->data[i]));
    }
    has_new_target = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    auto node = rclcpp::Node::make_shared("jaka_planner", options);

    // ✅ 核心修正 2：使用极低延迟的 QoS (Depth=1)
    // 只有最新的指令才会被接收，旧的直接丢掉，杜绝堆积延迟
    rclcpp::QoS qos_profile(1); 
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    auto joint_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/jaka_target_joints", 
        qos_profile, 
        jointTargetCallback
    );

    // MoveIt 初始化 (请确认组名是否为 "jaka_zu7" 或其他)
    // 如果报错 "Group not found"，请修改这里
    std::string PLANNING_GROUP = "jaka_zu20"; 
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // ✅ 核心修正 3：拉满速度和加速度
    // 遥操作要求响应快，不需要过于平滑
    move_group.setMaxVelocityScalingFactor(1.0);     
    move_group.setMaxAccelerationScalingFactor(1.0); 

    // 开启多线程处理 (保证回调函数能随时打断主循环)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    RCLCPP_INFO(rclcpp::get_logger("jaka_planner"), "🔥 极速异步控制模式已启动 (AsyncMove)");

    // 提高控制频率到 50Hz (每20ms检查一次)
    rclcpp::Rate rate(50); 
    
    while(rclcpp::ok())
    {
        if(has_new_target)
        {
            std::vector<double> target_copy;
            {
                std::lock_guard<std::mutex> lock(joint_mutex);
                target_copy = target_joint_values;
                has_new_target = false; // 取完数据立即复位标记
            }

            // ✅ 核心修正 4：使用 asyncMove() 异步执行
            // 不要用 move()！move() 会卡死线程直到运动结束。
            // asyncMove() 会立即触发运动并返回，允许我们在运动过程中
            // 随时发送新的指令来修正路径（实现"插队"）。
            move_group.setJointValueTarget(target_copy);
            move_group.asyncMove(); 
        }
        rate.sleep();
    }

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}