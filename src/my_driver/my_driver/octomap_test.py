import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math
import time

class OctomapProbe(Node):
    def __init__(self):
        super().__init__('octomap_probe')

        # 1. 订阅 OctoMap 的可视化话题 (读取方块)
        # 注意：这是 OctoMap Server 默认发布的用于 Rviz 显示的话题
        self.sub_map = self.create_subscription(
            MarkerArray,
            '/occupied_cells_vis_array', 
            self.map_callback,
            10
        )

        # 2. 发布“虚拟小球”给 Rviz 看
        self.pub_marker = self.create_publisher(Marker, '/virtual_probe', 10)

        # 3. 设置定时器，模拟小球运动
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz

        # 小球的初始状态
        self.ball_pos = [0.0, 0.0, 0.5] # x, y, z (在相机前方 0.5米)
        self.min_dist = 99.9            # 初始化为很远
        
        # 你的相机坐标系名字 (必须和 OctoMap 启动命令里的一致！)
        self.frame_id = "camera_depth_optical_frame" 

        self.get_logger().info("虚拟探针已启动！正在等待 OctoMap 数据...")

    def map_callback(self, msg):
        """
        这里处理 OctoMap 发过来的方块数据
        msg.markers 通常包含一个或多个 Marker，每个 Marker 里有一堆 points (方块中心点)
        """
        if not msg.markers:
            return

        min_d = 99.9
        
        # 遍历所有方块 (体素)
        # 注意：Python遍历大量点会慢，这是只是为了验证原理。
        # 实际工程中会用 C++ FCL 库。
        for marker in msg.markers:
            for point in marker.points:
                # 计算小球到该方块中心的欧氏距离
                dx = self.ball_pos[0] - point.x
                dy = self.ball_pos[1] - point.y
                dz = self.ball_pos[2] - point.z
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # 减去方块半径 (假设分辨率0.05，半径约0.025) 和小球半径(0.05)
                # 这样算的是“表面到表面”的距离
                dist = dist - 0.025 - 0.05 
                
                if dist < min_d:
                    min_d = dist
        
        self.min_dist = min_d
        # 在终端打印距离
        # self.get_logger().info(f"最近障碍物距离: {self.min_dist:.3f} 米")

    def control_loop(self):
        """
        控制循环：移动小球 + 发布可视化
        """
        # --- 1. 让小球前后移动 (模拟机器人在动) ---
        # 使用正弦波，让球在 Z 轴 (深度方向) 0.2米 到 1.0米 之间往复运动
        t = time.time()
        self.ball_pos[2] = 0.6 + 0.4 * math.sin(t) 
        self.ball_pos[0] = 0.0 # x 居中
        self.ball_pos[1] = 0.0 # y 居中

        # --- 2. 构建 Marker 消息 ---
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "probe"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position.x = self.ball_pos[0]
        marker.pose.position.y = self.ball_pos[1]
        marker.pose.position.z = self.ball_pos[2]
        
        # 设置大小 (直径 10cm)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # --- 3. 根据距离变色 (CBF 的雏形) ---
        marker.color.a = 1.0 # 不透明
        
        if self.min_dist < 0.1: # 距离小于 10cm -> 红色 (危险!)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.get_logger().warn(f"危险！距离: {self.min_dist:.3f}m", throttle_duration_sec=0.5)
        else: # 安全 -> 绿色
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        # 发布
        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = OctomapProbe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()