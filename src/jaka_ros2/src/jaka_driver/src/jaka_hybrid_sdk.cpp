#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <cmath> 

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/jkerr.h"

using namespace std::chrono_literals;

class HybridServoDirectNode : public rclcpp::Node {
public:
    HybridServoDirectNode() : Node("hybrid_servo_direct") {
        
        mode_.store(IDLE); // 显式 atomic 初始化
        is_robot_ready_.store(false); // 机器人就绪标志
        
        last_cmd_time_ = this->now(); 
        current_rpy_.resize(3, 0.0);
        latest_joint_vels_.resize(6, 0.0);

        std::string robot_ip = this->declare_parameter("ip", "192.168.1.11");
        RCLCPP_INFO(this->get_logger(), "🔌 正在通过 SDK 直连机械臂控制柜 (IP: %s)...", robot_ip.c_str());
        
        // 登录校验
        if (robot_.login_in(robot_ip.c_str(), false) != 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ 登录机械臂失败，请检查网络！");
            return; // 登录失败
        }
        
        int ret = robot_.servo_move_enable(true);
        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "✅ JAKA SDK 直连成功！伺服模式已就绪。");
            
            robot_.servo_move_use_joint_LPF(0.1);    
            //robot_.servo_speed_foresight(15, 0.03);  
            // 设置状态更新频率与阻塞超时，防止获取位姿卡死
            robot_.set_status_data_update_time_interval(100);
            robot_.set_block_wait_timeout(120);

            RCLCPP_INFO(this->get_logger(), "🛡️ 已向控制柜下发底层 LPF 滤波\n");
            is_robot_ready_.store(true); // 彻底就绪
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ SDK 开启伺服失败，错误码: %d", ret);
        }

        cart_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/jaka_cartesian_cmd", 10, 
            [this](const geometry_msgs::msg::Twist::SharedPtr msg){ 
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_twist_ = *msg;
                std::fill(latest_joint_vels_.begin(), latest_joint_vels_.end(), 0.0);
                last_cmd_time_ = this->now(); 
                mode_.store(CARTESIAN); 
            });

        joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/jaka_target_joints", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
                std::lock_guard<std::mutex> lock(data_mutex_);
                if(msg->data.size() == 6) latest_joint_vels_ = msg->data; 
                latest_twist_ = geometry_msgs::msg::Twist(); 
                last_cmd_time_ = this->now(); 
                mode_.store(JOINT_SINGLE); 
            });

        timer_ = this->create_wall_timer(8ms, std::bind(&HybridServoDirectNode::controlLoop, this));
    }

    ~HybridServoDirectNode() {
        if (is_robot_ready_.load()) {
            printf("\n"); 
            RCLCPP_INFO(this->get_logger(), "🛑 节点关闭，正在安全退出伺服模式并断开连接...");
            robot_.servo_move_enable(false);
            robot_.login_out();
        }
    }

private:
    enum ControlMode { IDLE, CARTESIAN, JOINT_SINGLE };
    std::atomic<ControlMode> mode_;
    std::atomic<bool> is_robot_ready_; // 机器人就绪标志，确保在未成功连接 SDK 时不执行控制逻辑
    
    std::mutex data_mutex_; 
    geometry_msgs::msg::Twist latest_twist_;
    std::vector<double> latest_joint_vels_;
    std::vector<double> current_rpy_;
    rclcpp::Time last_cmd_time_;
    
    const double dt_ = 0.008; // 125Hz

    JAKAZuRobot robot_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cart_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void controlLoop() {
        // 阻断非法下发：未连接成功直接返回
        if (!is_robot_ready_.load()) return;

        // 锁粒度优化：局部变量拷贝，实现光速解锁
        geometry_msgs::msg::Twist local_twist;
        std::vector<double> local_joint_vels;
        ControlMode local_mode;
        
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            local_twist = latest_twist_;
            local_joint_vels = latest_joint_vels_;
            local_mode = mode_.load(); 
            
            // 看门狗：如果超时，将全局与局部状态清零
            if ((this->now() - last_cmd_time_).seconds() > 0.1) {
                if (local_mode != IDLE) {
                    mode_.store(IDLE);
                    local_mode = IDLE;
                    latest_twist_ = geometry_msgs::msg::Twist();
                    std::fill(latest_joint_vels_.begin(), latest_joint_vels_.end(), 0.0);
                    printf("\n"); 
                    RCLCPP_INFO(this->get_logger(), "⏸️ 安全键松开，停止向 SDK 灌入数据...");
                }
            }
        } // 离开此大括号，锁已释放！后续的耗时网络 IO 不再阻塞 ROS 回调

        // =====================================
        // 运动指令解算与下发
        // =====================================
        if (local_mode == CARTESIAN) 
        {
            CartesianPose tcp_pose{}; // 全零初始化
            if (robot_.get_tcp_position(&tcp_pose) == 0) {
                current_rpy_[0] = tcp_pose.rpy.rx;
                current_rpy_[1] = tcp_pose.rpy.ry;
                current_rpy_[2] = tcp_pose.rpy.rz;
            }
            
            tf2::Quaternion q_current;
            q_current.setRPY(current_rpy_[0], current_rpy_[1], current_rpy_[2]);

            tf2::Vector3 vec_global(local_twist.linear.x * dt_, 
                                   local_twist.linear.y * dt_, 
                                   local_twist.linear.z * dt_);
            
            double dx_mm = std::clamp(vec_global.x() * 1000.0, -3.0, 3.0);
            double dy_mm = std::clamp(vec_global.y() * 1000.0, -3.0, 3.0);
            double dz_mm = std::clamp(vec_global.z() * 1000.0, -3.0, 3.0);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "🔍 [校验] 原始 vec_global (m): X=%.6f, Y=%.6f, Z=%.6f", 
                vec_global.x(), vec_global.y(), vec_global.z());

            tf2::Vector3 w_local(local_twist.angular.x, local_twist.angular.y, local_twist.angular.z);
            double angle = w_local.length() * dt_; 
            
            tf2::Quaternion q_delta_local;
            // 大于死区才生成旋转四元数
            if (angle > 1e-5) { 
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

            // 过滤极小死区
            if (std::abs(dx_mm) > 0.001 || std::abs(dy_mm) > 0.001 || std::abs(dz_mm) > 0.001 || angle > 1e-5) {
                CartesianPose cart_pose{}; // 全零初始化
                cart_pose.tran.x = dx_mm;
                cart_pose.tran.y = dy_mm;
                cart_pose.tran.z = dz_mm;
                cart_pose.rpy.rx = std::clamp(normalize_angle(d_roll), -0.01, 0.01);
                cart_pose.rpy.ry = std::clamp(normalize_angle(d_pitch), -0.01, 0.01);
                cart_pose.rpy.rz = std::clamp(normalize_angle(d_yaw), -0.01, 0.01);

                int ret = robot_.servo_p(&cart_pose, MoveMode::INCR);
                if (ret != 0) {
                    // 底层报错时除了记录日志，可立即在此处强制切入 IDLE
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "\n⚠️ 笛卡尔运动被底层拒绝 (错误码: %d)，请检查限位！", ret);
                }
            }
        }
        else if (local_mode == JOINT_SINGLE) 
        {
            bool has_movement = false;
            JointValue joint_pose{}; // 全零初始化

            for (size_t i = 0; i < 6; ++i) {
                double delta_rad = std::clamp(local_joint_vels[i] * dt_, -0.01, 0.01);
                joint_pose.jVal[i] = delta_rad;
                if (std::abs(delta_rad) > 0.0001) has_movement = true;
            }

            if (has_movement) {
                int ret = robot_.servo_j(&joint_pose, MoveMode::INCR);
                if (ret != 0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "\n⚠️ 关节运动被底层拒绝 (错误码: %d)", ret);
                }
            }
        }

        // =====================================
        // 单行刷新打印当前位姿
        // =====================================
        if (local_mode != IDLE) {
            CartesianPose actual_pose{};
            JointValue actual_joints{};
            
            if (robot_.get_tcp_position(&actual_pose) == 0 && robot_.get_joint_position(&actual_joints) == 0) {
                printf("\r[实时] XYZ(mm):%6.1f,%6.1f,%6.1f | RPY(°):%6.1f,%6.1f,%6.1f | J(°):%5.1f,%5.1f,%5.1f,%5.1f,%5.1f,%5.1f   ",
                    actual_pose.tran.x, actual_pose.tran.y, actual_pose.tran.z,
                    actual_pose.rpy.rx * 180.0 / M_PI, actual_pose.rpy.ry * 180.0 / M_PI, actual_pose.rpy.rz * 180.0 / M_PI,
                    actual_joints.jVal[0] * 180.0 / M_PI, actual_joints.jVal[1] * 180.0 / M_PI, actual_joints.jVal[2] * 180.0 / M_PI,
                    actual_joints.jVal[3] * 180.0 / M_PI, actual_joints.jVal[4] * 180.0 / M_PI, actual_joints.jVal[5] * 180.0 / M_PI);
                fflush(stdout); 
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HybridServoDirectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}