#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TfToPoseNode(Node):
    def __init__(self):
        super().__init__('tf_to_pose_bridge')
        self.publisher_ = self.create_publisher(PoseStamped, '/tcp_pose', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("TF to Pose Bridge 已启动！正在监听 base_link -> Link_06 ...")

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'Link_06', rclpy.time.Time())
            
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            
            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z
            msg.pose.orientation = t.transform.rotation
            
            self.publisher_.publish(msg)
            
        except Exception as e:
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