import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy.time # 导入时间模块

class RobotCapsuleVisualizer(Node):
    def __init__(self):
        super().__init__('robot_capsule_visualizer')
        
        # 参数部分保持不变...
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_capsules.link_0.parent_frame', 'base_link'),
                ('robot_capsules.link_0.start_point', [0.0, 0.0, 0.0]),
                ('robot_capsules.link_0.end_point', [0.0, 0.0, 0.28]),
                ('robot_capsules.link_0.radius', 0.12),
                ('robot_capsules.link_1.parent_frame', 'Link_01'),
                ('robot_capsules.link_1.start_point', [0.0, 0.0, 0.0]),
                ('robot_capsules.link_1.end_point', [0.0, 0.28, 0.0]),
                ('robot_capsules.link_1.radius', 0.15),
                ('robot_capsules.link_2.parent_frame', 'Link_02'),
                ('robot_capsules.link_2.start_point', [0.0, 0.0, -0.24]),
                ('robot_capsules.link_2.end_point', [0.897, 0.0, -0.24]),
                ('robot_capsules.link_2.radius', 0.14),
                ('robot_capsules.link_3.parent_frame', 'Link_03'),
                ('robot_capsules.link_3.start_point', [0.0, 0.0, -0.05]),
                ('robot_capsules.link_3.end_point', [0.0, 0.0, -0.28]),
                ('robot_capsules.link_3.radius', 0.10),
                ('robot_capsules.link_4.parent_frame', 'Link_03'),
                ('robot_capsules.link_4.start_point', [0.0, 0.0, -0.05]),
                ('robot_capsules.link_4.end_point', [0.74, 0.0, -0.05]),
                ('robot_capsules.link_4.radius', 0.10),
                ('robot_capsules.link_5.parent_frame', 'Link_05'),
                ('robot_capsules.link_5.start_point', [0.0, -0.05, 0.0]),
                ('robot_capsules.link_5.end_point', [0.0, 0.07, 0.0]),
                ('robot_capsules.link_5.radius', 0.08),
                ('robot_capsules.link_6.parent_frame', 'Link_05'),
                ('robot_capsules.link_6.start_point', [0.0, 0.0, 0.0]),
                ('robot_capsules.link_6.end_point', [0.0, 0.0, -0.18]),
                ('robot_capsules.link_6.radius', 0.08),
            ]
        )

        self.marker_pub = self.create_publisher(MarkerArray, '/robot_capsule_markers', 10)
        self.timer = self.create_timer(0.05, self.publish_capsules)
        self.get_logger().info("JAKA Zu20 修复版可视化节点已启动")

    def create_capsule_marker(self, id, frame_id, start, end, radius, name):
        markers = []
        
        zero_time = rclpy.time.Time().to_msg() 
        
        p1 = np.array(start)
        p2 = np.array(end)
        center = (p1 + p2) / 2.0
        direction = p2 - p1
        length = np.linalg.norm(direction)
        if length < 0.001: length = 0.001
        
        z_axis = np.array([0.0, 0.0, 1.0])
        target_dir = direction / length
        v = np.cross(z_axis, target_dir)
        c = np.dot(z_axis, target_dir)
        
        cyl = Marker()
        cyl.header.frame_id = frame_id
        cyl.header.stamp = zero_time # 使用零时间戳
        cyl.ns = name
        cyl.id = id * 10
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose.position.x, cyl.pose.position.y, cyl.pose.position.z = center.tolist()
        
        if np.linalg.norm(v) < 1e-6:
            cyl.pose.orientation.w = 1.0 if c > 0 else 0.0
            cyl.pose.orientation.y = 0.0 if c > 0 else 1.0
        else:
            s = np.linalg.norm(v)
            kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
            quat = R.from_matrix(rotation_matrix).as_quat()
            cyl.pose.orientation.x, cyl.pose.orientation.y, cyl.pose.orientation.z, cyl.pose.orientation.w = quat
            
        cyl.scale.x = radius * 2
        cyl.scale.y = radius * 2
        cyl.scale.z = length
        cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = 0.0, 1.0, 0.0, 0.4
        markers.append(cyl)

        for i, pt in enumerate([p1, p2]):
            sph = Marker()
            sph.header.frame_id = frame_id
            sph.header.stamp = zero_time # 使用零时间戳
            sph.ns = name
            sph.id = id * 10 + i + 1
            sph.type = Marker.SPHERE
            sph.pose.position.x, sph.pose.position.y, sph.pose.position.z = pt.tolist()
            sph.scale.x = sph.scale.y = sph.scale.z = radius * 2
            sph.color.r, sph.color.g, sph.color.b, sph.color.a = 0.8, 1.0, 0.0, 0.5
            markers.append(sph)
            
        return markers

    def publish_capsules(self):
        ma = MarkerArray()
        capsule_keys = ['link_0', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6']
        for i, key in enumerate(capsule_keys):
            try:
                prefix = f'robot_capsules.{key}.'
                frame = self.get_parameter(prefix + 'parent_frame').value
                start = self.get_parameter(prefix + 'start_point').value
                end = self.get_parameter(prefix + 'end_point').value
                radius = self.get_parameter(prefix + 'radius').value
                ma.markers.extend(self.create_capsule_marker(i, frame, start, end, radius, key))
            except Exception: continue
        self.marker_pub.publish(ma)

def main():
    rclpy.init()
    node = RobotCapsuleVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()