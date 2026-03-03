import sys
import rclpy
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import JointState, Image
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QSlider, 
                             QTextEdit, QGroupBox, QSpinBox, QDoubleSpinBox,
                             QLineEdit, QGridLayout)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
import pyqtgraph as pg
import subprocess
import cv2
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap

# ==========================================
# 1. ROS 2 工作线程 (使用组合模式代替多重继承)
# ==========================================
class Ros2Thread(QThread):
    # 定义 PyQt 信号，用于将 ROS 数据发送给主界面
    log_signal = pyqtSignal(str)
    distance_signal = pyqtSignal(float)
    cbf_warning_signal = pyqtSignal(bool)
    joint_states_signal = pyqtSignal(list)
    image_signal = pyqtSignal(object)  # 新增信号，传递OpenCV图像

    def __init__(self):
        super().__init__() 
        
        # 在内部实例化 Node
        self.node = rclpy.create_node('jaka_gcs_gui_node')
        
        # [订阅者] 订阅 CBF 最短距离 h(x)
        self.dist_sub = self.node.create_subscription(Float64, '/cbf/min_distance', self.dist_cb, 10)
        # [订阅者] 订阅机械臂关节状态
        self.joint_sub = self.node.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        # [订阅者] 订阅图像数据
        self.image_sub = self.node.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)
        
        # [发布者] 急停开关 & 参数调节
        self.estop_pub = self.node.create_publisher(Bool, '/estop_cmd', 10)
        self.gamma_pub = self.node.create_publisher(Float64, '/cbf/gamma_param', 10)

        self.bridge = CvBridge()

    def dist_cb(self, msg):
        self.distance_signal.emit(msg.data)
        # 如果距离小于 0.1m，发送 CBF 介入警告信号
        if msg.data < 0.1:
            self.cbf_warning_signal.emit(True)
        else:
            self.cbf_warning_signal.emit(False)

    def joint_cb(self, msg):
        # 假设前6个是 JAKA 的关节
        if len(msg.position) >= 6:
            self.joint_states_signal.emit(list(msg.position[:6]))

    def image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_signal.emit(cv_img)
        except Exception as e:
            self.log_signal.emit(f"[ERROR] 图像转换失败: {e}")

    def run(self):
        self.log_signal.emit("[INFO] ROS 2 GUI 节点已启动，等待数据...")
        rclpy.spin(self.node)

    def stop(self):
        self.log_signal.emit("[WARN] 正在关闭 ROS 2 节点...")
        self.node.destroy_node()


# ==========================================
# 2. PyQt5 主界面 (仅负责渲染和用户交互)
# ==========================================
class JakaGCSWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("JAKA 沉浸式遥操作地面站 (GCS)")
        self.resize(1100, 850) # 稍微加宽一点窗口，让并排显示更从容
        
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        self.init_ui()
        self.init_ros_thread()

    def init_ui(self):
        # --- 模块 1: 核心启停与状态监控 ---
        group1 = QGroupBox("1. 核心控制与急停 (Node & E-Stop)")
        layout1 = QHBoxLayout()
        
        self.btn_launch = QPushButton("启动遥操作 (Launch)")
        self.btn_launch.clicked.connect(self.launch_teleop)
        
        self.btn_estop = QPushButton("急停 (E-STOP)")
        self.btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 16px;")
        self.btn_estop.setMinimumHeight(50)
        self.btn_estop.clicked.connect(self.trigger_estop)
        
        layout1.addWidget(self.btn_launch)
        layout1.addWidget(self.btn_estop)
        group1.setLayout(layout1)

        # --- 模块 2: CBF 安全算法监控 ---
        group2 = QGroupBox("2. CBF 主动安全仪表盘 (CBF Dashboard)")
        layout2 = QVBoxLayout()
        
        self.plot_widget = pg.PlotWidget(title="胶囊体最短距离 h(x) 实时波形")
        self.plot_widget.setYRange(0, 1.0) 
        self.plot_widget.showGrid(x=True, y=True)
        self.dist_data = [0.0] * 100
        self.plot_curve = self.plot_widget.plot(self.dist_data, pen=pg.mkPen(color='g', width=2))
        layout2.addWidget(self.plot_widget)

        h_layout2 = QHBoxLayout()
        self.lbl_warning = QLabel("安全状态: 正常")
        self.lbl_warning.setStyleSheet("color: green; font-weight: bold; font-size: 14px;")
        
        self.slider_gamma = QSlider(Qt.Horizontal)
        self.slider_gamma.setRange(1, 100) 
        self.slider_gamma.setValue(10) 
        self.slider_gamma.valueChanged.connect(self.update_gamma)
        self.lbl_gamma = QLabel("CBF 激进程度 (Gamma): 1.0")

        h_layout2.addWidget(self.lbl_warning)
        h_layout2.addWidget(self.lbl_gamma)
        h_layout2.addWidget(self.slider_gamma)
        layout2.addLayout(h_layout2)
        group2.setLayout(layout2)

        # --- 模块 3: 视觉与建图 ---
        group3 = QGroupBox("3. 视觉与建图 (Vision & Mapping)")
        layout3 = QVBoxLayout()
        self.lbl_camera = QLabel("[摄像头视频流占位符]\n(建议另起 RViz2 渲染以防 GUI 卡顿)")
        self.lbl_camera.setAlignment(Qt.AlignCenter)
        self.lbl_camera.setStyleSheet("background-color: #1e1e1e; color: #888888; font-weight: bold;")
        self.lbl_camera.setMinimumHeight(150)
        
        self.btn_clear_map = QPushButton("清除 OctoMap 缓存")
        self.btn_clear_map.clicked.connect(lambda: self.log_msg("[INFO] 发送清除地图请求..."))
        layout3.addWidget(self.lbl_camera)
        layout3.addWidget(self.btn_clear_map)
        group3.setLayout(layout3)

        # --- 模块 4: 机械臂实时状态 ---
        group4 = QGroupBox("4. 机械臂实时状态 (Kinematics & Pose)")
        layout4 = QVBoxLayout()
        
        # 1. 关节角显示区 (横向铺开)
        layout4.addWidget(QLabel("关节空间 (Joint Space - rad):"))
        layout_joints = QGridLayout()
        self.joint_displays = []
        for i in range(6):
            lbl = QLabel(f"J{i+1}:")
            disp = QLineEdit("0.00")
            disp.setReadOnly(True)
            disp.setAlignment(Qt.AlignRight)
            disp.setStyleSheet("background-color: #e8e8e8; color: black; font-weight: bold;")
            self.joint_displays.append(disp)
            
            row = i // 6  # 变成单行显示6个，充分利用横向空间
            col = (i % 6) * 2
            layout_joints.addWidget(lbl, row, col)
            layout_joints.addWidget(disp, row, col+1)
        layout4.addLayout(layout_joints)

        # 2. 末端位姿显示区 (横向铺开)
        layout4.addWidget(QLabel("笛卡尔空间 (Task Space - m, rad):"))
        layout_pose = QGridLayout()
        pose_names = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        self.pose_displays = {}
        for i, name in enumerate(pose_names):
            lbl = QLabel(f"{name}:")
            disp = QLineEdit("0.00")
            disp.setReadOnly(True)
            disp.setAlignment(Qt.AlignRight)
            disp.setStyleSheet("background-color: #d1e7dd; color: black; font-weight: bold;") 
            self.pose_displays[name] = disp
            
            row = i // 6  # 单行显示6个
            col = (i % 6) * 2
            layout_pose.addWidget(lbl, row, col)
            layout_pose.addWidget(disp, row, col+1)
        layout4.addLayout(layout_pose)

        # 3. 摇杆灵敏度设置
        h_sens_layout = QHBoxLayout()
        h_sens_layout.addWidget(QLabel("摇杆死区 (%):"))
        self.spin_deadzone = QSpinBox()
        self.spin_deadzone.setValue(5)
        h_sens_layout.addWidget(self.spin_deadzone)
        
        h_sens_layout.addWidget(QLabel("最大速度(m/s):"))
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setValue(0.5)
        self.spin_speed.setSingleStep(0.1)
        h_sens_layout.addWidget(self.spin_speed)
        
        layout4.addLayout(h_sens_layout)
        group4.setLayout(layout4)

        # --- 模块 5: 日志控制台 ---
        group5 = QGroupBox("5. 系统日志控制台 (ROS 2 Logs)")
        layout5 = QVBoxLayout()
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setStyleSheet("background-color: #2b2b2b; color: #a9b7c6; font-family: Consolas;")
        layout5.addWidget(self.txt_log)
        group5.setLayout(layout5)

        # ==========================================
        # 组装主布局 (修改为 2 和 3 并排)
        # ==========================================
        h_layout_2_3 = QHBoxLayout()
        h_layout_2_3.addWidget(group2, 1) # 波形图占一半宽度
        h_layout_2_3.addWidget(group3, 1) # 视频流占一半宽度

        self.main_layout.addWidget(group1, 1)        
        self.main_layout.addLayout(h_layout_2_3, 4)  # 将并排的2和3放入主布局，占据主要高度
        self.main_layout.addWidget(group4, 2)        
        self.main_layout.addWidget(group5, 2)        

    def init_ros_thread(self):
        if not rclpy.ok():
            rclpy.init()
            
        self.ros_thread = Ros2Thread()
        self.ros_thread.log_signal.connect(self.log_msg)
        self.ros_thread.distance_signal.connect(self.update_distance_plot)
        self.ros_thread.cbf_warning_signal.connect(self.update_cbf_warning)
        self.ros_thread.joint_states_signal.connect(self.update_joint_states)
        self.ros_thread.image_signal.connect(self.update_camera_image)  # 新增连接
        
        self.ros_thread.start() 

    # --- 槽函数实现 ---
    def log_msg(self, msg):
        self.txt_log.append(msg)
        scrollbar = self.txt_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def launch_teleop(self):
        self.log_msg("[CMD] 尝试启动: ros2 launch teleop_driver teleop_hybrid.launch.py")

    def trigger_estop(self):
        self.log_msg("[ERROR] 触发急停！立即下发停止指令！")
        msg = Bool()
        msg.data = True
        self.ros_thread.estop_pub.publish(msg)

    def update_distance_plot(self, dist):
        self.dist_data = self.dist_data[1:] + [dist]
        self.plot_curve.setData(self.dist_data)

    def update_cbf_warning(self, is_warning):
        if is_warning:
            self.lbl_warning.setText("安全状态: CBF 介入中！")
            self.lbl_warning.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
            self.plot_curve.setPen(pg.mkPen(color='r', width=2)) 
        else:
            self.lbl_warning.setText("安全状态: 正常")
            self.lbl_warning.setStyleSheet("color: green; font-weight: bold; font-size: 14px;")
            self.plot_curve.setPen(pg.mkPen(color='g', width=2)) 

    def update_gamma(self):
        val = self.slider_gamma.value() / 10.0
        self.lbl_gamma.setText(f"CBF 激进程度 (Gamma): {val:.1f}")
        msg = Float64()
        msg.data = val
        self.ros_thread.gamma_pub.publish(msg)
        self.log_msg(f"[INFO] 更新 CBF Gamma 参数为: {val:.1f}")

    def update_joint_states(self, joints):
        for i, j_val in enumerate(joints):
            if i < 6:
                self.joint_displays[i].setText(f"{j_val:+.2f}")

    def update_camera_image(self, cv_img):
        # OpenCV BGR -> RGB
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_img).scaled(self.lbl_camera.width(), self.lbl_camera.height(), Qt.KeepAspectRatio)
        self.lbl_camera.setPixmap(pixmap)

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.quit()
        self.ros_thread.wait()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = JakaGCSWindow()
    window.show()
    sys.exit(app.exec_())