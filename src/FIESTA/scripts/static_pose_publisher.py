#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticPosePublisher(Node):
    def __init__(self):
        super().__init__('static_pose_publisher')

        # 发布 PoseStamped（给 FIESTA 的 transform 话题）
        self.pose_pub = self.create_publisher(PoseStamped, '/vins_estimator/camera_pose', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_pose)

        # 发布静态 TF（world → camera_depth_optical_frame，给 rviz 用）
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'camera_depth_optical_frame'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

def main():
    rclpy.init()
    node = StaticPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
