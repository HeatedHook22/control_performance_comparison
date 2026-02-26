# ================================
# Base OS: Ubuntu 22.04 (Jammy)
# ================================
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# ================================
# Basic Development Tools
# ================================
RUN apt update && apt install -y \
    sudo \
    curl \
    git \
    wget \
    build-essential \
    cmake \
    ninja-build \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    lsb-release \
    gnupg \
    software-properties-common \
    net-tools \
    iputils-ping

# ================================
# Setup ROS 2 Humble
# ================================
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-micro-ros-agent

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# ================================
# Gazebo Classic
# ================================
RUN apt install -y gazebo

# ================================
# PX4 Dependencies
# ================================
RUN apt install -y \
    python3-jinja2 \
    python3-numpy \
    python3-empy \
    python3-toml \
    exiftool \
    genromfs \
    libeigen3-dev \
    libopencv-dev

# ================================
# Clone PX4 Autopilot
# ================================
WORKDIR /workspace
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

WORKDIR /workspace/PX4-Autopilot

# Install PX4 Ubuntu dependencies (no NuttX, no extra sim tools)
RUN bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

WORKDIR /workspace

# Default shell
CMD ["/bin/bash"]