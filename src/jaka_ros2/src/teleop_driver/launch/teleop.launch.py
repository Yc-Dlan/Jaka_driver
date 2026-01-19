#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from jaka_msgs.srv import Move, ServoMoveEnable, ServoMove

import math

class JakaController(Node):
    def __init__(self):
        super().__init__('jaka_controller')
        
        # 参数配置
        self.declare_parameters(namespace='',
            parameters=[
                ('robot_ip', '192.168.1.10'),
                ('control_mode', 'joint'),  # joint或cartesian
                ('publish_rate', 20.0),
                ('max_linear_vel', 0.1),
                ('max_angular_vel', 0.3),
            ])
        
        self.robot_ip = self.get_parameter('robot_ip').value
        self.control_mode = self.get_parameter('control_mode').value
        self.is_servo_enabled = False
        self.current_joints = [0.0] * 6
        self.current_pose = [0.0] * 6
        
        # 创建服务客户端
        self.servo_enable_client = self.create_client(ServoMoveEnable, '/jaka_driver/servo_move_enable')
        self.joint_move_client = self.create_client(Move, '/jaka_driver/joint_move')
        self.linear_move_client = self.create_client(Move, '/jaka_driver/linear_move')
        self.servo_move_client = self.create_client(ServoMove, '/jaka_driver/servo_move')
        
        # 等待服务
        self.wait_for_services()
        
        # 启用伺服模式
        self.enable_servo_mode()
        
        # 订阅状态话题
        self.joint_sub = self.create_subscription(JointPosition, '/jaka_driver/joint_position', 
                                                 self.joint_callback, 10)
        self.tool_sub = self.create_subscription(ToolPosition, '/jaka_driver/tool_position',
                                               self.tool_callback, 10)
        
        # 订阅控制指令
        self.cmd_sub = self.create_subscription(TwistStamped, '/jaka_teleop/cmd_vel',
                                               self.cmd_callback, 10)
        
        # 控制定时器
        self.control_timer = self.create_timer(1.0/self.get_parameter('publish_rate').value,
                                             self.control_loop)
        
        self.get_logger().info(f'JAKA控制器已启动，模式: {self.control_mode}')

    def wait_for_services(self):
        """等待服务可用"""
        self.get_logger().info('等待JAKA驱动服务...')
        
        services = [
            (self.servo_enable_client, '/jaka_driver/servo_move_enable'),
            (self.joint_move_client, '/jaka_driver/joint_move'),
            (self.linear_move_client, '/jaka_driver/linear_move'),
            (self.servo_move_client, '/jaka_driver/servo_move')
        ]
        
        for client, service_name in services:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'服务 {service_name} 不可用')
            else:
                self.get_logger().info(f'服务 {service_name} 已连接')

    def enable_servo_mode(self):
        """启用伺服模式"""
        req = ServoMoveEnable.Request()
        req.enable = True
        future = self.servo_enable_client.call_async(req)
        future.add_done_callback(self.servo_enable_callback)

    def servo_enable_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.is_servo_enabled = True
                self.get_logger().info('伺服模式已启用')
            else:
                self.get_logger().error('启用伺服模式失败')
        except Exception as e:
            self.get_logger().error(f'伺服启用异常: {e}')

    def joint_callback(self, msg):
        """更新关节状态"""
        self.current_joints = list(msg.joint)
        self.get_logger().info(f'关节状态: {self.current_joints}', throttle_duration_sec=5.0)

    def tool_callback(self, msg):
        """更新工具位姿"""
        self.current_pose = list(msg.pose)
        self.get_logger().info(f'工具位姿: {self.current_pose}', throttle_duration_sec=5.0)

    def cmd_callback(self, msg):
        """处理控制指令"""
        if not self.is_servo_enabled:
            return
            
        self.last_cmd = msg.twist
        self.last_cmd_time = self.get_clock().now()
        
        # 调试输出
        self.get_logger().info(
            f'收到指令: 线性({msg.twist.linear.x:.3f}, {msg.twist.linear.y:.3f}, {msg.twist.linear.z:.3f}) '
            f'角速度({msg.twist.angular.x:.3f}, {msg.twist.angular.y:.3f}, {msg.twist.angular.z:.3f})',
            throttle_duration_sec=1.0
        )

    def control_loop(self):
        """主控制循环"""
        if not hasattr(self, 'last_cmd') or not self.is_servo_enabled:
            return
            
        # 检查指令超时（1秒）
        current_time = self.get_clock().now()
        if (current_time - self.last_cmd_time).nanoseconds > 1e9:
            return
        
        try:
            # 根据模式选择控制方法
            if self.control_mode == 'joint':
                self.send_joint_command()
            else:
                self.send_cartesian_command()
        except Exception as e:
            self.get_logger().error(f'控制异常: {e}')

    def send_joint_command(self):
        """发送关节运动指令"""
        max_linear = self.get_parameter('max_linear_vel').value
        max_angular = self.get_parameter('max_angular_vel').value
        dt = 1.0 / self.get_parameter('publish_rate').value
        
        # 计算关节速度（简化映射）
        joint_vels = [
            max(min(self.last_cmd.linear.x, max_linear), -max_linear) * 0.5,
            max(min(self.last_cmd.linear.y, max_linear), -max_linear) * 0.5, 
            max(min(self.last_cmd.linear.z, max_linear), -max_linear) * 0.5,
            max(min(self.last_cmd.angular.x, max_angular), -max_angular) * 0.3,
            max(min(self.last_cmd.angular.y, max_angular), -max_angular) * 0.3,
            max(min(self.last_cmd.angular.z, max_angular), -max_angular) * 0.3
        ]
        
        # 计算目标关节位置
        target_joints = [current + vel * dt for current, vel in zip(self.current_joints, joint_vels)]
        
        # 发送关节运动指令
        req = Move.Request()
        req.pose = target_joints
        req.mvvelo = 0.5  # 速度
        req.mvacc = 0.5    # 加速度
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0
        req.has_ref = False
        req.ref_joint = [0.0] * 6
        
        future = self.joint_move_client.call_async(req)
        future.add_done_callback(self.service_callback)

    def send_cartesian_command(self):
        """发送笛卡尔运动指令"""
        max_linear = self.get_parameter('max_linear_vel').value
        max_angular = self.get_parameter('max_angular_vel').value
        dt = 1.0 / self.get_parameter('publish_rate').value
        
        # 计算目标位姿
        target_pose = [
            self.current_pose[0] + max(min(self.last_cmd.linear.x, max_linear), -max_linear) * dt,
            self.current_pose[1] + max(min(self.last_cmd.linear.y, max_linear), -max_linear) * dt,
            self.current_pose[2] + max(min(self.last_cmd.linear.z, max_linear), -max_linear) * dt,
            self.current_pose[3] + max(min(self.last_cmd.angular.x, max_angular), -max_angular) * dt,
            self.current_pose[4] + max(min(self.last_cmd.angular.y, max_angular), -max_angular) * dt, 
            self.current_pose[5] + max(min(self.last_cmd.angular.z, max_angular), -max_angular) * dt
        ]
        
        # 发送线性运动指令
        req = Move.Request()
        req.pose = target_pose
        req.mvvelo = 100.0  # 速度
        req.mvacc = 100.0   # 加速度
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0
        req.has_ref = False
        req.ref_joint = [0.0] * 6
        
        future = self.linear_move_client.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        """服务调用回调"""
        try:
            response = future.result()
            if hasattr(response, 'success') and response.success:
                self.get_logger().info('运动指令执行成功', throttle_duration_sec=2.0)
            else:
                self.get_logger().warn('运动指令执行失败')
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {e}')

def main():
    rclpy.init()
    node = JakaController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('控制器关闭')
    finally:
        # 禁用伺服模式
        if node.is_servo_enabled:
            req = ServoMoveEnable.Request()
            req.enable = False
            node.servo_enable_client.call_async(req)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()