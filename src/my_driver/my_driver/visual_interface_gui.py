import sys
import math
import cv2
import rclpy
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped  
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QSlider, 
                             QTextEdit, QGroupBox, QSpinBox, QDoubleSpinBox,
                             QLineEdit, QGridLayout, QDialog, QFormLayout, QMessageBox) # 新增了 QDialog, QFormLayout, QMessageBox
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QProcess
import pyqtgraph as pg
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap, QFont

# ==========================================
# 0. 登录验证界面 (Login Dialog)
# ==========================================
class LoginWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("系统安全认证 - JAKA GCS")
        self.resize(350, 200)
        
        # 禁用右上角的问号提示按钮
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowContextHelpButtonHint)

        layout = QVBoxLayout(self)
        
        # 标题
        lbl_title = QLabel("JAKA 沉浸式遥操作地面站")
        lbl_title.setAlignment(Qt.AlignCenter)
        lbl_title.setFont(QFont("Arial", 14, QFont.Bold))
        lbl_title.setStyleSheet("color: #2c3e50; margin-bottom: 15px;")
        layout.addWidget(lbl_title)

        # 表单布局 (账号密码输入)
        form_layout = QFormLayout()
        self.user_input = QLineEdit()
        self.user_input.setPlaceholderText("请输入管理员账号")
        
        self.pass_input = QLineEdit()
        self.pass_input.setPlaceholderText("请输入密码")
        self.pass_input.setEchoMode(QLineEdit.Password) # 密码隐藏为黑点
        
        # 支持回车直接登录
        self.user_input.returnPressed.connect(self.check_login)
        self.pass_input.returnPressed.connect(self.check_login)

        form_layout.addRow("👤 账号:", self.user_input)
        form_layout.addRow("🔑 密码:", self.pass_input)
        layout.addLayout(form_layout)

        # 登录按钮
        self.btn_login = QPushButton("登  录 (Login)")
        self.btn_login.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; font-size: 14px; padding: 8px;")
        self.btn_login.clicked.connect(self.check_login)
        layout.addWidget(self.btn_login)

    def check_login(self):
        # ★ 在这里设置你的账号和密码 ★
        valid_user = "admin"
        valid_pass = "123456"

        if self.user_input.text() == valid_user and self.pass_input.text() == valid_pass:
            # 验证通过，关闭对话框并返回 Accepted 信号
            self.accept()
        else:
            # 验证失败，弹出警告并清空密码框
            QMessageBox.warning(self, "认证失败", "账号或密码错误，请重新输入！", QMessageBox.Ok)
            self.pass_input.clear()
            self.pass_input.setFocus()


# ==========================================
# 1. ROS 2 工作线程 (轻装上阵的订阅模式)
# ==========================================
class Ros2Thread(QThread):
    log_signal = pyqtSignal(str)
    distance_signal = pyqtSignal(float)
    cbf_warning_signal = pyqtSignal(bool)
    joint_states_signal = pyqtSignal(list)
    image_signal = pyqtSignal(object)
    pose_signal = pyqtSignal(list)  

    def __init__(self):
        super().__init__() 
        self.node = rclpy.create_node('jaka_gcs_gui_node')
        
        # [订阅者] 
        self.dist_sub = self.node.create_subscription(Float64, '/cbf/min_distance', self.dist_cb, 10)
        self.joint_sub = self.node.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.image_sub = self.node.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)
        self.pose_sub = self.node.create_subscription(PoseStamped, '/tcp_pose', self.pose_cb, 10)
        
        # [发布者] 
        self.estop_pub = self.node.create_publisher(Bool, '/estop_cmd', 10)
        self.gamma_pub = self.node.create_publisher(Float64, '/cbf/gamma_param', 10)

        self.bridge = CvBridge()
        self._is_running = True

    def pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        t0 = +2.0 * (qw * qx + qy * qz)
        t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
        rx = math.atan2(t0, t1)
        
        t2 = +2.0 * (qw * qy - qz * qx)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        ry = math.asin(t2)
        
        t3 = +2.0 * (qw * qz + qx * qy)
        t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
        rz = math.atan2(t3, t4)
        
        self.pose_signal.emit([x, y, z, rx, ry, rz])

    def dist_cb(self, msg):
        self.distance_signal.emit(msg.data)
        if msg.data < 0.1:
            self.cbf_warning_signal.emit(True)
        else:
            self.cbf_warning_signal.emit(False)

    def joint_cb(self, msg):
        if len(msg.position) >= 6:
            self.joint_states_signal.emit(list(msg.position[:6]))

    def image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_signal.emit(cv_img)
        except Exception as e:
            self.log_signal.emit(f"[ERROR] 图像转换失败: {e}")

    def run(self):
        self.log_signal.emit("[INFO] ROS 2 GUI 节点已启动，正在等待 /tcp_pose 数据...")
        while rclpy.ok() and self._is_running:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def stop(self):
        self.log_signal.emit("[WARN] 正在安全切断 ROS 2 内部订阅节点...")
        self._is_running = False  
        self.wait()               
        if self.node:
            self.node.destroy_node()


# ==========================================
# 2. PyQt5 主界面 
# ==========================================
class JakaGCSWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("JAKA 沉浸式遥操作地面站 (GCS)")
        self.resize(1100, 850) 
        
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        self.init_ui()
        self.init_processes()
        self.init_ros_thread()

    def init_ui(self):
        # --- 模块 1: 核心控制 ---
        group1 = QGroupBox("1. 核心控制与急停 (Node & E-Stop)")
        layout1 = QHBoxLayout()
        
        self.btn_launch = QPushButton("启动遥操作 (Launch)")
        self.btn_launch.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.btn_launch.clicked.connect(self.launch_teleop)
        
        self.btn_estop = QPushButton("急停 (E-STOP)")
        self.btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 16px;")
        self.btn_estop.setMinimumHeight(50)
        self.btn_estop.clicked.connect(self.trigger_estop)
        
        layout1.addWidget(self.btn_launch)
        layout1.addWidget(self.btn_estop)
        group1.setLayout(layout1)

        # --- 模块 2: CBF 仪表盘 ---
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

        # --- 模块 3: 视觉监控 ---
        group3 = QGroupBox("3. 视觉与建图 (Vision & Mapping)")
        layout3 = QVBoxLayout()
        
        self.btn_rviz = QPushButton("🚀 启动 RViz2 3D 监控视角")
        self.btn_rviz.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; font-size: 14px; padding: 10px;")
        self.btn_rviz.clicked.connect(self.launch_rviz)
        layout3.addWidget(self.btn_rviz)

        self.lbl_camera = QLabel("[摄像头视频流占位符]")
        self.lbl_camera.setAlignment(Qt.AlignCenter)
        self.lbl_camera.setStyleSheet("background-color: #1e1e1e; color: #888888; font-weight: bold;")
        self.lbl_camera.setMinimumHeight(150)
        
        self.btn_clear_map = QPushButton("清除 OctoMap 缓存")
        self.btn_clear_map.clicked.connect(lambda: self.log_msg("[INFO] 发送清除地图请求..."))
        layout3.addWidget(self.lbl_camera)
        layout3.addWidget(self.btn_clear_map)
        group3.setLayout(layout3)

        # --- 模块 4: 机械臂状态 ---
        group4 = QGroupBox("4. 机械臂实时状态 (Kinematics & Pose)")
        layout4 = QVBoxLayout()
        
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
            row = i // 6  
            col = (i % 6) * 2
            layout_joints.addWidget(lbl, row, col)
            layout_joints.addWidget(disp, row, col+1)
        layout4.addLayout(layout_joints)

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
            row = i // 6  
            col = (i % 6) * 2
            layout_pose.addWidget(lbl, row, col)
            layout_pose.addWidget(disp, row, col+1)
        layout4.addLayout(layout_pose)

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

        # --- 组装主布局 ---
        h_layout_2_3 = QHBoxLayout()
        h_layout_2_3.addWidget(group2, 1) 
        h_layout_2_3.addWidget(group3, 1) 

        self.main_layout.addWidget(group1, 1)        
        self.main_layout.addLayout(h_layout_2_3, 4)  
        self.main_layout.addWidget(group4, 2)        
        self.main_layout.addWidget(group5, 2)        

    def init_processes(self):
        self.launch_process = QProcess(self)
        self.launch_process.readyReadStandardOutput.connect(self.handle_stdout)
        self.launch_process.readyReadStandardError.connect(self.handle_stderr)
        self.launch_process.stateChanged.connect(self.handle_process_state)
        self.rviz_process = QProcess(self)

    def init_ros_thread(self):
        self.ros_thread = Ros2Thread()
        self.ros_thread.log_signal.connect(self.log_msg)
        self.ros_thread.distance_signal.connect(self.update_distance_plot)
        self.ros_thread.cbf_warning_signal.connect(self.update_cbf_warning)
        self.ros_thread.joint_states_signal.connect(self.update_joint_states)
        self.ros_thread.image_signal.connect(self.update_camera_image) 
        self.ros_thread.pose_signal.connect(self.update_pose_displays) 
        self.ros_thread.start() 

    # ==========================================
    # ★ 核心槽函数实现 ★
    # ==========================================
    def log_msg(self, msg):
        self.txt_log.append(msg)
        scrollbar = self.txt_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def launch_teleop(self):
        if self.launch_process.state() == QProcess.NotRunning:
            self.log_msg("[CMD] 正在启动遥操作节点: ros2 launch teleop_driver teleop_hybrid.launch.py")
            self.launch_process.start("ros2", ["launch", "teleop_driver", "teleop_hybrid.launch.py"])
            self.btn_launch.setText("停止遥操作 (Stop)")
            self.btn_launch.setStyleSheet("background-color: #ff9800; color: white; font-weight: bold; font-size: 14px;")
        else:
            self.log_msg("[CMD] 正在发送中止信号给遥操作节点...")
            self.launch_process.terminate() 
            self.launch_process.waitForFinished(2000) 
            if self.launch_process.state() != QProcess.NotRunning:
                self.launch_process.kill()

    def launch_rviz(self):
        if self.rviz_process.state() == QProcess.NotRunning:
            self.log_msg("[CMD] 正在唤醒独立的 RViz2 3D 监控引擎...")
            self.rviz_process.start("rviz2", []) 
            self.btn_rviz.setText("关闭 RViz2 监控")
            self.btn_rviz.setStyleSheet("background-color: #607D8B; color: white; font-weight: bold; font-size: 14px; padding: 10px;")
        else:
            self.log_msg("[CMD] 正在关闭 RViz2...")
            self.rviz_process.kill()
            self.rviz_process.waitForFinished(1000)
            self.btn_rviz.setText("🚀 启动 RViz2 3D 监控视角")
            self.btn_rviz.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; font-size: 14px; padding: 10px;")

    def handle_stdout(self):
        data = self.launch_process.readAllStandardOutput()
        stdout_text = bytes(data).decode("utf8", errors='replace').strip()
        if stdout_text:
            self.log_msg(f"[LAUNCH] {stdout_text}")

    def handle_stderr(self):
        data = self.launch_process.readAllStandardError()
        stderr_text = bytes(data).decode("utf8", errors='replace').strip()
        if stderr_text:
            self.log_msg(f"[LAUNCH ERR] {stderr_text}")

    def handle_process_state(self, state):
        if state == QProcess.NotRunning:
            self.btn_launch.setText("启动遥操作 (Launch)")
            self.btn_launch.setStyleSheet("font-weight: bold; font-size: 14px;")
            self.log_msg("[INFO] 遥操作节点已完全退出。")

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

    def update_pose_displays(self, pose_data):
        pose_names = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        for i, name in enumerate(pose_names):
            self.pose_displays[name].setText(f"{pose_data[i]:+.3f}")

    def update_camera_image(self, cv_img):
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_img).scaled(
            self.lbl_camera.width(), self.lbl_camera.height(), Qt.KeepAspectRatio)
        self.lbl_camera.setPixmap(pixmap)

    def closeEvent(self, event):
        if hasattr(self, 'launch_process') and self.launch_process.state() != QProcess.NotRunning:
            self.log_msg("[WARN] 正在强制结束后台 launch 进程...")
            self.launch_process.kill()
            self.launch_process.waitForFinished(1000)
            
        if hasattr(self, 'rviz_process') and self.rviz_process.state() != QProcess.NotRunning:
            self.rviz_process.kill()
            self.rviz_process.waitForFinished(1000)
            
        self.ros_thread.stop()
        event.accept()

# ==========================================
# 3. 全局入口
# ==========================================
if __name__ == '__main__':
    # 1. 优先初始化 ROS 2
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    
    # 2. 实例化并弹出登录界面 (阻塞)
    login_dialog = LoginWindow()
    
    # 3. 判断登录结果
    if login_dialog.exec_() == QDialog.Accepted:
        # 如果密码正确，才加载主控界面
        window = JakaGCSWindow()
        window.show()
        exit_code = app.exec_()
    else:
        # 如果点击右上角关闭或者验证失败退出，直接结束程序
        exit_code = 0

    # 4. 退出清理
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)