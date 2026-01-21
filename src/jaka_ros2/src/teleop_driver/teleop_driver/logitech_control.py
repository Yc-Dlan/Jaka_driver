#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
JAKA 机械臂全能遥操作节点 (Final Safe Version)
包含：
1. 三模式切换 (关节组 / 笛卡尔-工具系 / 单关节)
2. 极低延迟 QoS
3. [修复] 笛卡尔模式速度缩放，防止乱飞
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

# ===================== 🎮 硬件映射 =====================
AXIS_LR       = 0   # 左右 (X)
AXIS_FB       = 1   # 前后 (Y)
AXIS_TWIST    = 2   # 旋转 (Z)
AXIS_THROTTLE = 3   # 节流阀
AXIS_HAT_X    = 4   # 苦力帽左右
AXIS_HAT_Y    = 5   # 苦力帽上下

BTN_SHIFT     = 0   # [扳机键] 功能随模式变化
BTN_DEADMAN   = 1   # [拇指键] 安全锁 (必须按住)
BTN_MODE      = 2   # [顶部键] 切换模式

# ===================== ⚙️ 参数配置 =====================
ARM_JOINT_NUM = 6
BASE_SPEED = 0.04       # 关节模式基础速度 (弧度)
WRIST_SPEED_BOOST = 3.0 # 手腕关节加速倍率
LIMIT_RAD = 6.28        # 软件限位 ±360度
DEADZONE = 0.05         # 摇杆死区

# 模式枚举
MODE_JOINT_GROUP = 0
MODE_CARTESIAN   = 1
MODE_SINGLE      = 2
MODE_NAMES = ["关节组联动", "笛卡尔(工具系)", "单关节微调"]

class JakaJoystickTeleop(Node):
    def __init__(self):
        super().__init__('jaka_joystick_teleop')

        # QoS 配置：丢弃旧数据，保证实时性
        low_latency_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # 状态变量
        self.virtual_joints = None
        self.current_mode = MODE_JOINT_GROUP
        self.selected_joint_idx = 0 
        self.last_btn_mode_state = 0
        self.last_hat_x = 0.0

        # 通信接口
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, low_latency_qos)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 发布关节指令 (模式 0 & 2)
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/jaka_target_joints', low_latency_qos)
        # 发布笛卡尔指令 (模式 1)
        self.cart_cmd_pub = self.create_publisher(Twist, '/jaka_cartesian_cmd', low_latency_qos)

        self.get_logger().info('✅ 安全版控制器就绪。请按 [Button 2] 切换模式。')

    def safe_get(self, lst, idx, default=0.0):
        return lst[idx] if idx < len(lst) else default

    def clamp(self, val):
        return max(-LIMIT_RAD, min(val, LIMIT_RAD))

    def joint_state_callback(self, msg):
        # 仅在启动时同步一次真实位置
        if self.virtual_joints is None and len(msg.position) >= ARM_JOINT_NUM:
            self.virtual_joints = list(msg.position)[:ARM_JOINT_NUM]
            self.get_logger().info('位置校准完成')

    def joy_callback(self, msg):
        try:
            # 1. 安全锁检查
            if not self.safe_get(msg.buttons, BTN_DEADMAN):
                return 
            if self.virtual_joints is None:
                return

            # 2. 模式切换 (上升沿检测)
            btn_mode = self.safe_get(msg.buttons, BTN_MODE)
            if btn_mode == 1 and self.last_btn_mode_state == 0:
                self.current_mode = (self.current_mode + 1) % 3
                self.get_logger().info(f'🔄 切换模式: [{MODE_NAMES[self.current_mode]}]')
            self.last_btn_mode_state = btn_mode

            # 3. 速度计算
            raw_throttle = self.safe_get(msg.axes, AXIS_THROTTLE)
            speed_ratio = (raw_throttle * -1 + 1.0) / 2.0 
            current_speed = BASE_SPEED * (0.5 + speed_ratio * 2.0)

            # 4. 读取摇杆 (基础量)
            raw_x = self.safe_get(msg.axes, AXIS_LR)
            raw_y = self.safe_get(msg.axes, AXIS_FB)
            raw_twist = self.safe_get(msg.axes, AXIS_TWIST)

            val_x = 0.0 if abs(raw_x) < DEADZONE else raw_x 
            val_y = 0.0 if abs(raw_y) < DEADZONE else raw_y 
            val_twist = 0.0 if abs(raw_twist) < DEADZONE else raw_twist 

            is_shift = self.safe_get(msg.buttons, BTN_SHIFT)

            # ================= 核心分流逻辑 =================

            # ---【模式 0: 关节组联动 (Body/Wrist)】---
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
                
                self.publish_joints()

            # ---【模式 1: 笛卡尔空间 (工具坐标系)】---
            elif self.current_mode == MODE_CARTESIAN:
                twist = Twist()
                
                # 🛑 [关键修复] 笛卡尔位置单位是米(m)。
                # 0.04 * 0.02 = 0.0008m/帧 = 0.8mm/帧。
                # 50Hz 下速度约为 4cm/s。这是一个安全的可控速度。
                # 之前如果用了 1.5，速度会达到 3m/s，导致乱飞。
                lin_scale = current_speed * 0.02 
                ang_scale = current_speed * 0.5  # 角速度单位是弧度，可以稍大

                if not is_shift:
                    # === 位置控制 (Linear) - 工具系 ===
                    # 前推 -> Z+ (进给)
                    twist.linear.z = val_y * lin_scale
                    # 左右 -> Y- (横移)
                    twist.linear.y = -val_x * lin_scale 
                    # 旋转 -> X+ (升降)
                    twist.linear.x = val_twist * lin_scale
                else:
                    # === 姿态控制 (Angular) - 绕工具轴旋转 ===
                    twist.angular.y = val_y * ang_scale   # Pitch
                    twist.angular.z = -val_x * ang_scale  # Roll
                    twist.angular.x = val_twist * ang_scale # Yaw

                self.cart_cmd_pub.publish(twist)

            # ---【模式 2: 单关节微调】---
            elif self.current_mode == MODE_SINGLE:
                # 苦力帽选关节
                hat_x = self.safe_get(msg.axes, AXIS_HAT_X)
                if abs(hat_x) > 0.5 and self.last_hat_x == 0.0:
                    d = 1 if hat_x < 0 else -1
                    self.selected_joint_idx = (self.selected_joint_idx + d) % ARM_JOINT_NUM
                    self.get_logger().info(f'👉 选中关节: J{self.selected_joint_idx + 1}')
                self.last_hat_x = hat_x

                # 摇杆 Y 轴控制
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