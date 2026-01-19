#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# ===================== 配置项：按需修改 =====================
CONTROL_MODE = "ARM"  # "ARM"=机械臂(关节角度)，"BASE"=底盘/无人机(Twist速度)
ARM_JOINT_NUM = 6     # JAKA机械臂是6轴
LINEAR_SCALE_DEF = 0.1
ANGULAR_SCALE_DEF = 0.5
DEADZONE_DEF = 0.1
# ======================================================================

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        
        # 声明参数 + 兜底默认值
        self.linear_scale = self.declare_parameter('linear_scale', LINEAR_SCALE_DEF).value
        self.angular_scale = self.declare_parameter('angular_scale', ANGULAR_SCALE_DEF).value
        self.deadzone = self.declare_parameter('deadzone', DEADZONE_DEF).value

        # 根据控制模式创建发布者
        if CONTROL_MODE == "ARM":
            self.cmd_pub = self.create_publisher(Float64MultiArray, '/jaka_target_joints', 10)
            self.get_logger().info(f'✅ 机械臂模式启动 | 发布关节角度到: /jaka_target_joints')
        else:
            self.cmd_pub = self.create_publisher(Twist, '/jaka_teleop/cmd_vel', 10)
            self.get_logger().info(f'✅ 底盘/无人机模式启动 | 发布速度指令到: /jaka_teleop/cmd_vel')

        # 订阅摇杆数据
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 防抖：缓存上一次按键状态 - 初始化为空列表
        self.last_buttons = []
        self.get_logger().info(f'飞行摇杆遥操作节点已启动 | 死区:{self.deadzone} | 线速度缩放:{self.linear_scale}')

    # 安全访问摇杆轴，索引越界返回0.0，永不崩溃
    def safe_get_axis(self, axes_list, index, default=0.0):
        return axes_list[index] if index < len(axes_list) else default

    # 安全访问按键，索引越界返回0，永不崩溃
    def safe_get_button(self, btn_list, index, default=0):
        return btn_list[index] if index < len(btn_list) else default

    # 死区过滤逻辑
    def apply_deadzone(self, value):
        return value if abs(value) > self.deadzone else 0.0

    def joy_callback(self, msg):
        try:
            # ========== 核心修复：将array.array 转为Python原生list ==========
            joy_axes = list(msg.axes)
            joy_buttons = list(msg.buttons)
            
            # 读取摇杆数据 + 安全防护 + 死区过滤
            filtered_axes = [self.apply_deadzone(ax) for ax in joy_axes]
            # 按键防抖初始化：长度不一致则重置
            self.last_buttons = self.last_buttons if len(self.last_buttons) == len(joy_buttons) else [0]*len(joy_buttons)

            # 分模式处理指令
            if CONTROL_MODE == "ARM":
                # 机械臂模式：摇杆轴 → 6关节弧度值 (适配JAKA机械臂)
                joint_msg = Float64MultiArray()
                joint_angles = [0.0]*ARM_JOINT_NUM
                # 摇杆轴映射到6关节 (可根据手感调整轴索引)
                joint_angles[0] = self.safe_get_axis(filtered_axes, 1) * 1.57 + 1.57  # 关节1:0~3.14rad
                joint_angles[1] = self.safe_get_axis(filtered_axes, 0) * 1.57 + 1.57  # 关节2:0~3.14rad
                joint_angles[2] = self.safe_get_axis(filtered_axes, 2) * -1.57 - 1.57 # 关节3:-3.14~0rad
                joint_angles[3] = self.safe_get_axis(filtered_axes, 3) * 1.57 + 1.57  # 关节4:0~3.14rad
                joint_angles[4] = self.safe_get_axis(filtered_axes, 4) * 1.57 + 1.57  # 关节5:0~3.14rad
                joint_angles[5] = self.safe_get_axis(filtered_axes, 5) * 1.57 + 1.57  # 关节6:0~3.14rad
                joint_msg.data = joint_angles
                self.cmd_pub.publish(joint_msg)
                self.get_logger().info(f'📢 发布关节角度: {[round(x,3) for x in joint_angles]}')
            
            else:
                # 底盘模式：保留原逻辑，修复所有BUG
                cmd_vel = Twist()
                # 左摇杆控制平移
                cmd_vel.linear.x = self.safe_get_axis(filtered_axes, 1) * self.linear_scale
                cmd_vel.linear.y = self.safe_get_axis(filtered_axes, 0) * self.linear_scale
                # 肩部按键控制Z轴
                btn4 = self.safe_get_button(joy_buttons, 4)
                btn5 = self.safe_get_button(joy_buttons, 5)
                cmd_vel.linear.z = (btn4 - btn5) * self.linear_scale

                # 右摇杆控制姿态角速度
                cmd_vel.angular.x = self.safe_get_axis(filtered_axes, 4) * self.angular_scale
                cmd_vel.angular.y = self.safe_get_axis(filtered_axes, 3) * self.angular_scale
                # 按钮控制偏航角速度
                btn0 = self.safe_get_button(joy_buttons, 0)
                btn1 = self.safe_get_button(joy_buttons, 1)
                cmd_vel.angular.z = (btn0 - btn1) * self.angular_scale

                self.cmd_pub.publish(cmd_vel)
                self.get_logger().info(f'📢 发布速度指令: linear({round(cmd_vel.linear.x,2)},{round(cmd_vel.linear.y,2)},{round(cmd_vel.linear.z,2)}) | angular({round(cmd_vel.angular.x,2)},{round(cmd_vel.angular.y,2)},{round(cmd_vel.angular.z,2)})')

            # ✅ 修复核心：更新按键状态，用转换后的list赋值即可，无需copy()
            self.last_buttons = joy_buttons

        except Exception as e:
            self.get_logger().error(f'回调函数异常: {str(e)}', throttle_duration_sec=1)

def main():
    rclpy.init(args=None)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到退出信号，正在关闭节点...')
    except Exception as e:
        node.get_logger().error(f'节点运行异常: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
