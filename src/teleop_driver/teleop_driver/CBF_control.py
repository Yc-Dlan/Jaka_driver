import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker, MarkerArray
import math

class CBFController(Node):
    def __init__(self):
        super().__init__('cbf_controller')

        # 1. 订阅 OctoMap (环境感知)
        self.sub_map = self.create_subscription(
            MarkerArray, '/occupied_cells_vis_array', self.map_callback, 10)
        
        # 2. 订阅 摇杆 (用户意图)
        self.sub_joy = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        # 3. 发布 虚拟小球 (可视化)
        self.pub_marker = self.create_publisher(Marker, '/virtual_probe', 10)

        # 4. 定时器 (控制频率 20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        # --- 系统状态 ---
        self.ball_pos = [0.0, 0.0, 0.5]  # 初始位置
        self.min_dist = 99.9             # 最近障碍物距离 h(x)
        self.v_nominal = 0.0             # 用户想要的原始速度
        self.v_safe = 0.0                # 经过 CBF 修正后的速度

        # --- CBF 参数 (关键！) ---
        self.gamma = 2.0                 # 刹车激进程度 (越大刹得越晚)
        self.d_safe = 0.10               # 安全余量 (10cm 就不让走了)

        self.get_logger().info("CBF 控制器就绪！请推动摇杆...")

    def map_callback(self, msg):
        """ 计算最近距离 h(x) """
        if not msg.markers: return
        min_d = 99.9
        # 简化版计算：遍历体素 (实际项目请用 FCL 库)
        for marker in msg.markers:
            for point in marker.points:
                dist = math.sqrt(
                    (self.ball_pos[0] - point.x)**2 +
                    (self.ball_pos[1] - point.y)**2 +
                    (self.ball_pos[2] - point.z)**2
                )
                # 减去体素半径和小球半径
                dist = dist - 0.025 - 0.05
                if dist < min_d: min_d = dist
        self.min_dist = min_d

    def joy_callback(self, msg):
        """ 读取用户意图 v_nom """
        # 假设 axes[1] 是前后推杆 (Logitech 3D Pro)
        # 映射范围：[-1, 1] -> [-0.5m/s, 0.5m/s]
        self.v_nominal = msg.axes[1] * 0.5 

    def control_loop(self):
        """ 核心算法：CBF 过滤器 """
        dt = 0.05
        
        # --- 步骤 1: 获取当前的安全指标 h ---
        # h < 0 说明已经撞上了，h > 0 说明安全
        h = self.min_dist - self.d_safe

        # --- 步骤 2: CBF 公式 ---
        # 只有当用户想“冲向”障碍物 (v > 0 且前方有障碍) 时才修正
        # 这里简化为一维运动 (Z轴方向)
        
        # 假设障碍物在前方 (Z轴正方向)
        if self.v_nominal > 0: 
            # CBF 约束: v <= gamma * h
            # 如果 h 很大(很远)，速度限制就很大(没限制)
            # 如果 h 很小(很近)，速度限制就接近 0
            limit = self.gamma * h
            
            # 只有当限速比用户想跑的速度慢时，才生效
            if limit < 0: limit = 0 # 不允许负数（反向）
            
            self.v_safe = min(self.v_nominal, limit)
            
            if self.v_safe < self.v_nominal:
                print(f"CBF 介入! 原速: {self.v_nominal:.2f}, 修正后: {self.v_safe:.2f}, 距离: {h:.3f}")
        else:
            # 如果是往回拉 (远离障碍物)，完全不限制
            self.v_safe = self.v_nominal

        # --- 步骤 3: 积分更新位置 ---
        # 在真实机器人上，这里是发布 /cmd_vel 给控制器
        # 在这里，我们直接移动虚拟球
        self.ball_pos[2] += self.v_safe * dt
        
        # 限制一下球别跑太远
        self.ball_pos[2] = max(0.0, min(self.ball_pos[2], 2.0))

        # --- 步骤 4: 可视化 ---
        self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "camera_depth_optical_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = self.ball_pos[2]
        marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1
        marker.color.a = 1.0
        
        # 变色逻辑保留：CBF介入时变黄，危险时变红，正常绿
        if self.min_dist < self.d_safe:
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0 # 红
        elif self.v_safe < self.v_nominal: 
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0 # 黄 (被限制中)
        else:
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0 # 绿

        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = CBFController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()