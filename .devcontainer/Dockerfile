FROM osrf/ros:humble-desktop-full

# 定义用户和用户组
ARG USERNAME=yc-dlan
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 安装相关包
ARG ROS_DISTRO=humble

RUN apt-get update \
    && apt-get install -y sudo vim nautilus curl lsb-release gnupg git wget cmake build-essential

# 创建用户组和用户
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # 配置环境变量
    && echo "$USERNAME:'" | chpasswd \
    # 允许用户使用 sudo
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME \
    # 使新用户的 `.bashrc` 文件生效
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

USER root

# 安装MoveIt 2
RUN apt-get update \
    && apt-get install -y ros-humble-moveit

# 安装Gazebo Fortress
# 添加Gazebo Fortress软件源
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 安装Ignition Fortress和相关依赖
RUN apt-get update \
    && apt-get install -y ignition-fortress

# 安装ROS 2集成（ros-gz Bridge）
RUN apt-get install -y ros-humble-ros-gz

# 安装ign_ros2_control软件包
RUN apt-get update \
    && apt-get install -y ros-humble-ign-ros2-control ros-humble-moveit-visual-tools

RUN apt-get update \
    && apt-get install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gz-ros2-control
    
RUN apt-get update \
    && apt-get install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

RUN apt-get update \
    && apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at v4l-utils usbutils ros-humble-realsense2-camera

RUN mkdir -p lib \
    && cd lib \ 
    && git clone https://github.com/realsenseai/librealsense.git \
    && cd librealsense \
    && ./scripts/setup_udev_rules.sh \
    && mkdir build && cd build \
    && cmake -DFORCE_RSUSB_BACKEND=true ../ -DBUILD_EXAMPLES=true \
    && sudo make uninstall && make clean && make -j$(($(nproc)-1)) && sudo make install

# 安装八叉树地图
RUN apt-get update \
    && apt-get install -y ros-humble-octomap-server ros-humble-octomap-rviz-plugins

# 清理apt缓存以减少镜像大小
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 切回yc-dlan
USER yc-dlan

# 安装fish插件（切换到普通用户）
RUN curl -L https://github.com/oh-my-fish/oh-my-fish/raw/master/bin/install | tee install_omf > /dev/null || true \
    && fish install_omf --noninteractive || true \
    && fish -c "omf install bass" || true \
    && rm install_omf || true

# 复制fish配置
COPY packages/fish /home/${USERNAME}/.config/fish

WORKDIR /home/${USERNAME}

CMD [ "/bin/bash" ]