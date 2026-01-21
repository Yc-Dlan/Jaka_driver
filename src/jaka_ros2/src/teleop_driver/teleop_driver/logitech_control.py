#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray, Int32
from geometry_msgs.msg import Twist

# ===================== 🎮 硬件映射 =====================
AXIS_LR       = 0   # 左右
AXIS_FB       = 1   # 前后
AXIS_TWIST    = 2   # 旋转
AXIS_THROTTLE = 3   # 节流阀
AXIS_HAT_X    = 4   # 苦力帽左右 (根据你的摇杆确认 ID)
AXIS_HAT_Y    = 5   # 苦力帽上下

BTN_SHIFT     = 0   # 扳机键 (换挡)
BTN_DEADMAN   = 1   # 拇指键 (使能)
BTN_MODE      = 2   # 【新】模式切换键

# ===================== ⚙️ 参数 =====================
ARM_JOINT_NUM = 6
BASE_SPEED = 0.04 
WRIST_SPEED_BOOST = 3.0 
LIMIT_RAD = 6.28  
DEADZONE = 0.05

# 模式定义
MODE_JOINT_GROUP = 0  # 原有模式 (身体/手腕)
MODE_CARTESIAN   = 1  # 笛卡尔 (XYZ/RPY)
MODE_SINGLE      = 2  # 单关节 (逐个控制)
MODE_NAMES = ["关节组联动", "笛卡尔空间", "单关节微调"]

class JakaJoystickTeleop(Node):
    def __init__(self):
        super().__init__('jaka_joystick_teleop')

        # QoS
        low_latency_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # 状态变量
        self.virtual_joints = None
        self.current_mode = MODE_JOINT_GROUP
        self.selected_joint_idx = 0 # 单关节模式下选中的关节 (0-5)
        
        # 按键边沿检测
        self.last_btn_mode_state = 0
        self.last_hat_x = 0.0

        # 通信
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, low_latency_qos)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 发布关节指令 (用于 模式0 和 模式2)
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/jaka_target_joints', low_latency_qos)
        
        # 发布笛卡尔指令 (用于 模式1) - 发送 Twist (线速度+角速度)
        self.cart_cmd_pub = self.create_publisher(Twist, '/jaka_cartesian_cmd', low_latency_qos)

        self.get_logger().info('✅ 三合一控制器就绪。按 Button 2 切换模式。')

    def safe_get(self, lst, idx, default=0.0):
        return lst[idx] if idx < len(lst) else default

    def clamp(self, val):
        return max(-LIMIT_RAD, min(val, LIMIT_RAD))

    def joint_state_callback(self, msg):
        # 仅初始化时对齐
        if self.virtual_joints is None and len(msg.position) >= ARM_JOINT_NUM:
            self.virtual_joints = list(msg.position)[:ARM_JOINT_NUM]
            self.get_logger().info('初始位置校准完成')

    def joy_callback(self, msg):
        try:
            # 1. 安全锁
            if not self.safe_get(msg.buttons, BTN_DEADMAN):
                return 
            if self.virtual_joints is None:
                return

            # 2. 模式切换逻辑 (检测上升沿)
            btn_mode_curr = self.safe_get(msg.buttons, BTN_MODE)
            if btn_mode_curr == 1 and self.last_btn_mode_state == 0:
                self.current_mode = (self.current_mode + 1) % 3
                self.get_logger().info(f'🔄 切换模式: [{MODE_NAMES[self.current_mode]}]')
            self.last_btn_mode_state = btn_mode_curr

            # 3. 计算通用速度
            raw_throttle = self.safe_get(msg.axes, AXIS_THROTTLE)
            speed_ratio = (raw_throttle * -1 + 1.0) / 2.0 
            current_speed = BASE_SPEED * (0.5 + speed_ratio * 2.0)

            # 4. 读取基础摇杆轴
            raw_x = self.safe_get(msg.axes, AXIS_LR)
            raw_y = self.safe_get(msg.axes, AXIS_FB)
            raw_twist = self.safe_get(msg.axes, AXIS_TWIST)

            val_x = 0.0 if abs(raw_x) < DEADZONE else raw_x 
            val_y = 0.0 if abs(raw_y) < DEADZONE else raw_y 
            val_twist = 0.0 if abs(raw_twist) < DEADZONE else raw_twist 

            is_shift = self.safe_get(msg.buttons, BTN_SHIFT)

            # ================= 核心分流逻辑 =================

            # ---【模式 0: 关节组联动 (原有逻辑)】---
            if self.current_mode == MODE_JOINT_GROUP:
                if not is_shift: # 身体 J1-J3
                    self.virtual_joints[0] = self.clamp(self.virtual_joints[0] + val_x * current_speed)
                    self.virtual_joints[1] = self.clamp(self.virtual_joints[1] - val_y * current_speed)
                    self.virtual_joints[2] = self.clamp(self.virtual_joints[2] - val_twist * current_speed * 1.5)
                else: # 手腕 J4-J6
                    bs = current_speed * WRIST_SPEED_BOOST
                    self.virtual_joints[3] = self.clamp(self.virtual_joints[3] + val_x * bs)
                    self.virtual_joints[4] = self.clamp(self.virtual_joints[4] - val_y * bs)
                    self.virtual_joints[5] = self.clamp(self.virtual_joints[5] + val_twist * bs)
                
                # 发布关节数据
                self.publish_joints()

            # ---【模式 1: 笛卡尔空间控制】---
            elif self.current_mode == MODE_CARTESIAN:
                # 构造 Twist 消息发送给 C++
                twist = Twist()
                # 速度缩放：笛卡尔移动需要把单位换算合适 (m/s)
                linear_scale = current_speed * 2.0  # 约 0.1m/s
                angular_scale = current_speed * 3.0 # 约 0.15rad/s

                if not is_shift:
                    # 不按扳机：控制位置 (XYZ)
                    # 左右推->Y轴，前后推->X轴，旋转->Z轴 (符合大多数习惯，可自行修改)
                    twist.linear.y = -val_x * linear_scale
                    twist.linear.x = val_y * linear_scale
                    twist.linear.z = val_twist * linear_scale
                else:
                    # 按住扳机：控制姿态 (Roll/Pitch/Yaw)
                    twist.angular.y = -val_x * angular_scale
                    twist.angular.x = val_y * angular_scale
                    twist.angular.z = val_twist * angular_scale

                self.cart_cmd_pub.publish(twist)
                # 注意：笛卡尔模式下，virtual_joints 可能会过时，
                # 但只要切回关节模式，C++ 的 joint_states 会再次校准它(虽然有延迟)，或者我们可以接受跳变
                # 更完美的做法是 C++ 实时回传 IK 解算后的关节给 Python，这里为简化暂不处理

            # ---【模式 2: 单关节独立控制】---
            elif self.current_mode == MODE_SINGLE:
                # 使用苦力帽左右 (Axis 4) 切换选中的关节
                hat_x = self.safe_get(msg.axes, AXIS_HAT_X)
                if abs(hat_x) > 0.5 and self.last_hat_x == 0.0:
                    direction = 1 if hat_x < 0 else -1 # 根据苦力帽方向
                    self.selected_joint_idx = (self.selected_joint_idx + direction) % ARM_JOINT_NUM
                    self.get_logger().info(f'👉 选中关节: J{self.selected_joint_idx + 1}')
                self.last_hat_x = hat_x

                # 使用摇杆 Y 轴 (前后) 控制该关节
                # 速度给慢一点，方便微调
                single_speed = current_speed * 0.8
                self.virtual_joints[self.selected_joint_idx] = self.clamp(
                    self.virtual_joints[self.selected_joint_idx] - val_y * single_speed
                )
                
                # 发布关节数据
                self.publish_joints()

        except Exception as e:
            self.get_logger().error(f'Joy Error: {e}')

    def publish_joints(self):
        msg = Float64MultiArray()
        msg.data = self.virtual_joints
        self.joint_cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = JakaJoystickTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()