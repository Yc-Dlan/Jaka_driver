#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TfToPoseNode(Node):
    def __init__(self):
        super().__init__('tf_to_pose_bridge')
        # 创建发布者，专门发布算好的笛卡尔坐标
        self.publisher_ = self.create_publisher(PoseStamped, '/tcp_pose', 10)
        
        # 初始化 TF 监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 50Hz 高频查询并发布
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("TF to Pose Bridge 已启动！正在监听 base_link -> Link_06 ...")

    def timer_callback(self):
        try:
            # ★ 这里的名字已经严格替换为你系统里真实的 Link 名称 ★
            t = self.tf_buffer.lookup_transform('base_link', 'Link_06', rclpy.time.Time())
            
            # 打包成 PoseStamped 消息
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            
            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z
            msg.pose.orientation = t.transform.rotation
            
            # 发布出去
            self.publisher_.publish(msg)
            
        except Exception as e:
            # TF还没准备好时静默忽略
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TfToPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()