#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

# ===================== 🎮 硬件映射 =====================
AXIS_LR       = 0   
AXIS_FB       = 1   
AXIS_TWIST    = 2   
AXIS_THROTTLE = 3   
AXIS_HAT_X    = 4   
AXIS_HAT_Y    = 5   

BTN_SHIFT     = 0   # [扳机键] 功能随模式变化
BTN_DEADMAN   = 1   # [拇指键] 安全锁 (必须按住)
BTN_MODE      = 2   # [顶部键] 切换模式

# ===================== ⚙️ 参数配置 =====================
ARM_JOINT_NUM = 6
BASE_SPEED = 0.02       # 关节模式基础步长 (稍微调小，真机实测为主)
WRIST_SPEED_BOOST = 3.0 
LIMIT_RAD = 6.28        
DEADZONE = 0.05         

MODE_JOINT_GROUP = 0
MODE_CARTESIAN   = 1
MODE_SINGLE      = 2
MODE_NAMES = ["关节组联动", "笛卡尔(工具系)", "单关节微调"]

class JakaJoystickTeleop(Node):
    def __init__(self):
        super().__init__('jaka_joystick_teleop')

        low_latency_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        self.real_joints = None     # 真实物理关节角
        self.virtual_joints = None  # 内部计算用的虚拟关节角
        
        self.current_mode = MODE_JOINT_GROUP
        self.selected_joint_idx = 0 
        self.last_btn_mode_state = 0
        self.last_hat_x = 0.0

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, low_latency_qos)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/jaka_target_joints', low_latency_qos)
        self.cart_cmd_pub = self.create_publisher(Twist, '/jaka_cartesian_cmd', low_latency_qos)

        self.get_logger().info('✅ 前端指令器就绪 (防漂移修复版)。请按 [Button 2] 切换模式。')

    def safe_get(self, lst, idx, default=0.0):
        return lst[idx] if idx < len(lst) else default

    def clamp(self, val):
        return max(-LIMIT_RAD, min(val, LIMIT_RAD))

    def joint_state_callback(self, msg):
        # 时刻记录真实的机械臂位置
        if len(msg.position) >= ARM_JOINT_NUM:
            self.real_joints = list(msg.position)[:ARM_JOINT_NUM]
            # 启动时初始化
            if self.virtual_joints is None:
                self.virtual_joints = list(self.real_joints)
                self.get_logger().info('✅ 初始位置校准完成')

    def joy_callback(self, msg):
        try:
            if self.real_joints is None or self.virtual_joints is None:
                return

            # 【核心修复】：如果没有按住安全锁，时刻让虚拟角度同步为真实的物理角度！
            if not self.safe_get(msg.buttons, BTN_DEADMAN):
                self.virtual_joints = list(self.real_joints)
                return 

            # 模式切换
            btn_mode = self.safe_get(msg.buttons, BTN_MODE)
            if btn_mode == 1 and self.last_btn_mode_state == 0:
                self.current_mode = (self.current_mode + 1) % 3
                self.get_logger().info(f'🔄 切换模式: [{MODE_NAMES[self.current_mode]}]')
            self.last_btn_mode_state = btn_mode

            raw_throttle = self.safe_get(msg.axes, AXIS_THROTTLE)
            speed_ratio = (raw_throttle * -1 + 1.0) / 2.0 
            current_speed = BASE_SPEED * (0.5 + speed_ratio * 2.0)

            raw_x = self.safe_get(msg.axes, AXIS_LR)
            raw_y = self.safe_get(msg.axes, AXIS_FB)
            raw_twist = self.safe_get(msg.axes, AXIS_TWIST)

            val_x = 0.0 if abs(raw_x) < DEADZONE else raw_x 
            val_y = 0.0 if abs(raw_y) < DEADZONE else raw_y 
            val_twist = 0.0 if abs(raw_twist) < DEADZONE else raw_twist 
            is_shift = self.safe_get(msg.buttons, BTN_SHIFT)

            if self.current_mode == MODE_JOINT_GROUP:
                if not is_shift: 
                    self.virtual_joints[0] = self.clamp(self.virtual_joints[0] + val_x * current_speed)
                    self.virtual_joints[1] = self.clamp(self.virtual_joints[1] - val_y * current_speed)
                    self.virtual_joints[2] = self.clamp(self.virtual_joints[2] - val_twist * current_speed * 1.5)
                else: 
                    bs = current_speed * WRIST_SPEED_BOOST
                    self.virtual_joints[3] = self.clamp(self.virtual_joints[3] + val_x * bs)
                    self.virtual_joints[4] = self.clamp(self.virtual_joints[4] - val_y * bs)
                    self.virtual_joints[5] = self.clamp(self.virtual_joints[5] + val_twist * bs)
                self.publish_joints()

            elif self.current_mode == MODE_CARTESIAN:
                twist = Twist()
                lin_scale = current_speed * 0.02 
                ang_scale = current_speed * 0.5  

                if not is_shift:
                    twist.linear.z = val_y * lin_scale
                    twist.linear.y = -val_x * lin_scale 
                    twist.linear.x = val_twist * lin_scale
                else:
                    twist.angular.y = val_y * ang_scale   
                    twist.angular.z = -val_x * ang_scale  
                    twist.angular.x = val_twist * ang_scale 
                self.cart_cmd_pub.publish(twist)

            elif self.current_mode == MODE_SINGLE:
                hat_x = self.safe_get(msg.axes, AXIS_HAT_X)
                if abs(hat_x) > 0.5 and self.last_hat_x == 0.0:
                    d = 1 if hat_x < 0 else -1
                    self.selected_joint_idx = (self.selected_joint_idx + d) % ARM_JOINT_NUM
                    self.get_logger().info(f'👉 选中关节: J{self.selected_joint_idx + 1}')
                self.last_hat_x = hat_x

                single_speed = current_speed * 0.8
                self.virtual_joints[self.selected_joint_idx] = self.clamp(
                    self.virtual_joints[self.selected_joint_idx] - val_y * single_speed
                )
                self.publish_joints()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def publish_joints(self):
        msg = Float64MultiArray()
        msg.data = self.virtual_joints
        self.joint_cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = JakaJoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()