import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import collections

MAX_LIN_VEL = 0.05   # 笛卡尔线速度: m/s
MAX_ANG_VEL = 0.05   # 笛卡尔角速度: rad/s
MAX_JOINT_VEL = 0.1  # 单关节角速度: rad/s

AXIS_X, AXIS_Y, AXIS_TWIST, AXIS_THROTTLE = 0, 1, 2, 3
BTN_TRIGGER = 0
BTN_DEADMAN = 1 # 安全键
BTN_MODE = 2    # 模式切换
BTN_PREV_J = 4  # 上一个关节 (LB)
BTN_NEXT_J = 5  # 下一个关节 (RB)

class JakaManualTeleop(Node):
    def __init__(self):
        super().__init__('jaka_manual_teleop')
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.cart_pub = self.create_publisher(Twist, '/jaka_cartesian_cmd', 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, '/jaka_target_joints', 10)

        self.mode = "CARTESIAN"
        self.active_joint_index = 0
        
        self.last_btns = {} 
        
        # 滑动平均窗口，消除抖动
        self.axes_history = collections.deque(maxlen=10)
        
        self.get_logger().info("✅ 单关节/笛卡尔混合控制已启动")
        self.get_logger().info("🕹️  操作: [LB/RB]切换关节 | [Btn2]切换模式 | [Y轴]驱动")

    def joy_cb(self, msg):
        # 如果没有按下安全键，清空历史缓存并退出
        if not msg.buttons[BTN_DEADMAN]: 
            self.axes_history.clear()
            return

        self.handle_buttons(msg)

        # 存入原始摇杆数据
        self.axes_history.append((msg.axes[AXIS_X], msg.axes[AXIS_Y], msg.axes[AXIS_TWIST]))
        
        # 计算滑动平均值 (低通滤波)
        avg_x = sum(h[0] for h in self.axes_history) / len(self.axes_history)
        avg_y = sum(h[1] for h in self.axes_history) / len(self.axes_history)
        avg_twist = sum(h[2] for h in self.axes_history) / len(self.axes_history)

        # 速度倍率
        ratio = ((-msg.axes[AXIS_THROTTLE] + 1) / 2) * 0.8 + 0.2
        
        # 摇杆死区
        val_x = avg_x if abs(avg_x) > 0.05 else 0.0
        val_y = avg_y if abs(avg_y) > 0.05 else 0.0
        val_twist = avg_twist if abs(avg_twist) > 0.1 else 0.0

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
            joint_vels = [0.0] * 6
            joint_vels[self.active_joint_index] = val_y * MAX_JOINT_VEL * ratio
            
            cmd = Float64MultiArray()
            cmd.data = joint_vels
            self.joint_pub.publish(cmd)

    def handle_buttons(self, msg):
        def is_pressed(idx):
            return msg.buttons[idx] == 1 and self.last_btns.get(idx, 0) == 0

        # 切换模式
        if is_pressed(BTN_MODE):
            self.mode = "JOINT_SINGLE" if self.mode == "CARTESIAN" else "CARTESIAN"
            self.get_logger().info(f"🔄 切换模式: {self.mode}")

        # 切换关节 
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