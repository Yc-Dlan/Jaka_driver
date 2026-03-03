# Jaka_driver
This warehouse will store the functional packages related to the Jaka robotic arm, as well as materials for scientific research and training.

## 项目简介
Jaka_driver 是一个用于 Jaka 机械臂的 ROS2 功能包集合，包含驱动、消息、规划、仿真等模块，适用于科研、教学和工业应用。

## 目录结构
- `src/`：源代码目录，包含各功能包
- `build/`：编译生成文件
- `install/`：安装后的文件
- `packages/`：依赖包及资源
- `log/`：日志文件

## 安装方法
1. 克隆仓库：
   ```bash
   git clone https://github.com/Yc-Dlan/Jaka_driver.git
   ```
2. 进入目录并编译：
   ```bash
   cd Jaka_driver
   colcon build --symlink-install
   ```
3. 环境配置：
   ```bash
   source install/setup.bash
   ```

## 使用说明
- 启动驱动节点：
  ```bash
  ros2 launch jaka_driver jaka_driver.launch.py
  ```
- 更多功能请参考各功能包下的 README 或文档。

## 功能模块
- **jaka_driver**：Jaka机械臂驱动节点，负责与硬件通信，实现运动控制。
- **jaka_msgs**：自定义消息类型，便于ROS2节点间数据交互。
- **jaka_planner**：运动规划模块，支持路径生成与轨迹跟踪。
- **jaka_description**：机械臂模型与URDF描述文件，支持仿真与可视化。
- **jaka_zu20_moveit_config**：MoveIt配置包，支持机械臂运动规划与碰撞检测。
- **my_driver/teleop_driver**：扩展驱动或遥控模块。

## 支持环境
- **操作系统**：Ubuntu 20.04/22.04
- **ROS版本**：ROS2 Foxy/Humble
- **硬件**：Jaka机械臂系列（如Zu 20等）
- **依赖**：colcon、MoveIt、Realsense（如有）、相关驱动包

## 示例代码
以下为Python节点控制机械臂的简单示例：
```python
import rclpy
from jaka_msgs.msg import JointCommand

rclpy.init()
node = rclpy.create_node('jaka_demo')
pub = node.create_publisher(JointCommand, '/jaka_driver/joint_command', 10)
cmd = JointCommand()
cmd.position = [0.0, 0.5, 0.0, -0.5, 0.0, 0.0]
pub.publish(cmd)
```

## 常见问题 FAQ
- **Q: 编译报错找不到依赖？**
  A: 请确认已安装所有ROS2依赖，并执行 `rosdep install --from-paths src --ignore-src -r -y`。
- **Q: 启动节点后无响应？**
  A: 检查硬件连接，确保机械臂已上电并与主机网络畅通。
- **Q: 如何添加自定义功能包？**
  A: 在 `src/` 目录下新建包，并在 `CMakeLists.txt` 和 `package.xml` 中添加依赖。

## 社区与资源
- [Jaka官网](https://www.jakarobotics.com/)
- [ROS2官方文档](https://docs.ros.org/en/foxy/index.html)
- [MoveIt官方文档](https://moveit.ros.org/)
- [Issues反馈](https://github.com/Yc-Dlan/Jaka_driver/issues)

---
如需进一步帮助，请在 Issues 区留言或邮件联系维护者。

## SOMETHING TO DO 
- 软件部分，带手柄测试加了界面后的操作是否有问题，虚拟仿真最后一部分了
- 硬件部分：对于深度相机基本驱动和输出没什么问题，只剩下上实物做标定 + 测试数据传输稳定性；对于激光雷达，驱动和输出没什么问题，同样剩上实际标定，后续还要涉及融合效果，不过也可以塞在一起。
- 算法部分：避障主要选择控制障碍函数去做约束和纠正，要不要加一些优化部分再说