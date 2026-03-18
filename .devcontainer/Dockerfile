FROM osrf/ros:humble-desktop

# 1. 安装原有的 Nav2, Cartographer，并加入 Cyclone DDS 和视觉依赖
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-teleop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-usb-cam \
    ros-humble-cv-bridge \
    python3-pip \
    nano \
    && rm -rf /var/lib/apt/lists/*

# 2. 安装 Python 依赖 (YOLOv8 和无头版 OpenCV 以防 GUI 报错)
RUN pip3 install --no-cache-dir ultralytics opencv-python-headless "numpy<2"

# 3. 设置核心环境变量 (注入了 Cyclone DDS 和 LDS-01 雷达型号)
ENV TURTLEBOT3_MODEL=burger
ENV LDS_MODEL=LDS-01
ENV ROS_DOMAIN_ID=106
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 4. 配置终端自动 source 环境
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc

WORKDIR /root/robot3_ws