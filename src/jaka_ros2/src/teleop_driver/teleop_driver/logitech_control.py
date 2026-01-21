#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
JAKA 机械臂低延迟遥操作节点 (最终修正版)
特性：
1. QoS 深度设为 1 (丢弃旧数据，只发最新指令)
2. 虚拟位置积分控制 (解决弹簧回弹风险)
3. 动态速度调节 + 手腕极速模式
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
import math

# ===================== 🎮 硬件映射 (基于您的设备) =====================
AXIS_LR       = 0   # 左右 -> J1 / J4
AXIS_FB       = 1   # 前后 -> J2 / J5
AXIS_TWIST    = 2   # 旋转 -> J3 / J6
AXIS_THROTTLE = 3   # 节流阀

BTN_SHIFT     = 0   # 扳机键 (手腕模式切换)
BTN_DEADMAN   = 1   # 拇指键 (必须按住才能动)

# ===================== ⚙️ 核心参数调优 =====================
ARM_JOINT_NUM = 6
# 基础步长：调大此值可提高整体响应速度 (建议 0.03 - 0.08)
BASE_SPEED = 0.04 
# 手腕加速倍率：手腕关节转动惯量小，给 3.0 倍速才跟手
WRIST_SPEED_BOOST = 3.0 
# 软件限位：放宽到 ±360度 (6.28 rad) 以防止撞虚拟墙
LIMIT_RAD = 6.28  
DEADZONE = 0.05
# ===========================================================

class JakaJoystickTeleop(Node):
    def __init__(self):
        super().__init__('jaka_joystick_teleop')

        # --- 1. 低延迟 QoS 配置 ---
        # 关键：只保留最后 1 条数据 (KeepLast=1)，旧数据直接丢弃
        # 这能防止网络卡顿后，机械臂疯狂执行积压的旧指令
        low_latency_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- 2. 内部状态 ---
        self.virtual_joints = None  # 积分控制的核心：记录"理论目标位置"

        # --- 3. 通信接口 ---
        # 订阅真实状态 (用于初始对齐)
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', # 如有前缀请修改，例如 '/jaka_zu7/joint_states'
            self.joint_state_callback, 
            low_latency_qos
        )

        # 订阅摇杆
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # 发布指令
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/jaka_target_joints', 
            low_latency_qos # 应用低延迟策略
        )

        self.get_logger().info('✅ Python端就绪：已启用极速模式 (QoS Depth=1)')

    def safe_get(self, lst, idx, default=0.0):
        return lst[idx] if idx < len(lst) else default

    def clamp(self, val):
        return max(-LIMIT_RAD, min(val, LIMIT_RAD))

    def joint_state_callback(self, msg):
        # 仅在启动时进行一次对齐，将虚拟位置同步为真实位置
        # 之后就不再受真实位置延迟的影响，完全由摇杆控制虚拟积分
        if self.virtual_joints is None and len(msg.position) >= ARM_JOINT_NUM:
            self.virtual_joints = list(msg.position)[:ARM_JOINT_NUM]
            self.get_logger().info(f'校准初始位置完成: {[round(x,2) for x in self.virtual_joints]}')

    def joy_callback(self, msg):
        try:
            # [安全锁] 没按拇指键，直接跳过
            if not self.safe_get(msg.buttons, BTN_DEADMAN):
                return 
            
            # [未校准] 等待接收第一帧 joint_states
            if self.virtual_joints is None:
                self.get_logger().warn('等待机械臂状态数据...', throttle_duration_sec=2)
                return

            # --- 1. 计算速度 (节流阀) ---
            # 映射范围：Axis3 [-1.0 ~ 1.0] -> 倍率 [0.5 ~ 2.5]
            raw_throttle = self.safe_get(msg.axes, AXIS_THROTTLE)
            speed_ratio = (raw_throttle * -1 + 1.0) / 2.0 
            current_speed = BASE_SPEED * (0.5 + speed_ratio * 2.0)

            # --- 2. 读取输入 ---
            raw_x = self.safe_get(msg.axes, AXIS_LR)
            raw_y = self.safe_get(msg.axes, AXIS_FB)
            raw_twist = self.safe_get(msg.axes, AXIS_TWIST)

            val_x = 0.0 if abs(raw_x) < DEADZONE else raw_x 
            val_y = 0.0 if abs(raw_y) < DEADZONE else raw_y 
            val_twist = 0.0 if abs(raw_twist) < DEADZONE else raw_twist 

            # --- 3. 积分控制逻辑 ---
            is_wrist_mode = self.safe_get(msg.buttons, BTN_SHIFT)

            if not is_wrist_mode:
                # === 身体模式 (J1-J3) ===
                # J1(左右), J2(前后-反向), J3(旋转-反向+加速)
                self.virtual_joints[0] = self.clamp(self.virtual_joints[0] + val_x * current_speed)
                self.virtual_joints[1] = self.clamp(self.virtual_joints[1] - val_y * current_speed)
                self.virtual_joints[2] = self.clamp(self.virtual_joints[2] - val_twist * current_speed * 1.5)
            else:
                # === 手腕模式 (J4-J6) ===
                # 应用加速倍率
                bs = current_speed * WRIST_SPEED_BOOST
                self.virtual_joints[3] = self.clamp(self.virtual_joints[3] + val_x * bs)
                self.virtual_joints[4] = self.clamp(self.virtual_joints[4] - val_y * bs)
                self.virtual_joints[5] = self.clamp(self.virtual_joints[5] + val_twist * bs)

            # --- 4. 发布极低延迟指令 ---
            msg_out = Float64MultiArray()
            msg_out.data = self.virtual_joints
            self.cmd_pub.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

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