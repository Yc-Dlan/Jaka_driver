#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time

# === 参数配置 ===
MAX_LIN_VEL = 0.15   # 笛卡尔线速度: m/s
MAX_ANG_VEL = 0.4    # 笛卡尔角速度: rad/s
MAX_JOINT_VEL = 0.5  # 单关节角速度: rad/s

# 罗技手柄映射
AXIS_X, AXIS_Y, AXIS_TWIST, AXIS_THROTTLE = 0, 1, 2, 3
BTN_TRIGGER = 0
BTN_DEADMAN = 1 # 安全键
BTN_MODE = 2    # 模式切换
BTN_PREV_J = 4  # [新] 上一个关节 (LB)
BTN_NEXT_J = 5  # [新] 下一个关节 (RB)

class JakaManualTeleop(Node):
    def __init__(self):
        super().__init__('jaka_manual_teleop')
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.cart_pub = self.create_publisher(Twist, '/jaka_cartesian_cmd', 10)
        # 注意：这里发布的将是"关节速度"向量，而不是位置
        self.joint_pub = self.create_publisher(Float64MultiArray, '/jaka_target_joints', 10)

        self.mode = "CARTESIAN"
        self.active_joint_index = 0 # 当前选中的关节 (0~5)
        
        # 按键边沿检测
        self.last_btns = {} 
        
        self.get_logger().info("✅ 单关节/笛卡尔混合控制已启动")
        self.get_logger().info("🕹️  操作: [LB/RB]切换关节 | [Btn2]切换模式 | [Y轴]驱动")

    def joy_cb(self, msg):
        if not msg.buttons[BTN_DEADMAN]: return

        # --- 1. 处理按键事件 (上升沿检测) ---
        self.handle_buttons(msg)

        # --- 2. 数据预处理 ---
        # 速度倍率
        ratio = ((-msg.axes[AXIS_THROTTLE] + 1) / 2) * 0.8 + 0.2
        
        # 摇杆死区
        val_x = msg.axes[AXIS_X] if abs(msg.axes[AXIS_X]) > 0.05 else 0.0
        val_y = msg.axes[AXIS_Y] if abs(msg.axes[AXIS_Y]) > 0.05 else 0.0
        val_twist = msg.axes[AXIS_TWIST] if abs(msg.axes[AXIS_TWIST]) > 0.1 else 0.0

        # --- 3. 模式分发 ---
        if self.mode == "CARTESIAN":
            twist = Twist()
            if not msg.buttons[BTN_TRIGGER]:
                twist.linear.y = val_y * MAX_LIN_VEL * ratio
                twist.linear.x = -val_x * MAX_LIN_VEL * ratio
                twist.linear.z = val_twist * MAX_LIN_VEL * ratio
            else:
                twist.angular.y = val_y * MAX_ANG_VEL * ratio
                twist.angular.z = -val_x * MAX_ANG_VEL * ratio
                twist.angular.x = val_twist * MAX_ANG_VEL * ratio
            self.cart_pub.publish(twist)

        elif self.mode == "JOINT_SINGLE":
            # 构建 6维 速度向量
            joint_vels = [0.0] * 6
            
            # 只给当前选中的关节赋值
            # 注意：val_y 推上去是正，拉下来是负
            joint_vels[self.active_joint_index] = val_y * MAX_JOINT_VEL * ratio
            
            cmd = Float64MultiArray()
            cmd.data = joint_vels
            self.joint_pub.publish(cmd)

    def handle_buttons(self, msg):
        # 辅助函数：检测按键是否刚刚被按下
        def is_pressed(idx):
            return msg.buttons[idx] == 1 and self.last_btns.get(idx, 0) == 0

        # 切换模式
        if is_pressed(BTN_MODE):
            self.mode = "JOINT_SINGLE" if self.mode == "CARTESIAN" else "CARTESIAN"
            self.get_logger().info(f"🔄 切换模式: {self.mode}")

        # 切换关节 (仅在关节模式下有效)
        if self.mode == "JOINT_SINGLE":
            if is_pressed(BTN_NEXT_J):
                self.active_joint_index = (self.active_joint_index + 1) % 6
                self.get_logger().info(f"🦾 选中关节: J{self.active_joint_index + 1}")
            
            if is_pressed(BTN_PREV_J):
                self.active_joint_index = (self.active_joint_index - 1) % 6
                self.get_logger().info(f"🦾 选中关节: J{self.active_joint_index + 1}")

        # 更新按键状态
        for i in [BTN_MODE, BTN_PREV_J, BTN_NEXT_J]:
            self.last_btns[i] = msg.buttons[i]

def main():
    rclpy.init()
    rclpy.spin(JakaManualTeleop())
    rclpy.shutdown()

if __name__ == '__main__':
    main()