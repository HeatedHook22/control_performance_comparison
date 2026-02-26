# ================================
# Base OS: Ubuntu 22.04 (Jammy)
# ================================
FROM ubuntu:22.04

# Set environment variables to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# ================================
# 1. Basic tools & ROS 2 Repo setup
# ================================
RUN apt update && apt install -y \
    sudo curl git wget build-essential cmake ninja-build \
    python3-pip lsb-release gnupg2 software-properties-common \
    net-tools iputils-ping \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# ================================
# 2. Install ROS 2 Humble & Simulation Tools
# ================================
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    gazebo \
    libgazebo-dev

# ================================
# 3. PX4 Dependencies & uXRCE Agent
# ================================
RUN apt install -y \
    python3-jinja2 python3-numpy python3-empy python3-toml \
    python3-packaging \
    exiftool genromfs libeigen3-dev libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    astyle

# Install Python packages not available in apt, and pin numpy to < 2.0 to avoid matplotlib crashes
RUN pip3 install "numpy<2" pyros-genmsg

# Build uXRCE-DDS Agent (Required for PX4-ROS2 communication)
WORKDIR /opt
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent && mkdir build && cd build \
    && cmake .. && make && make install && ldconfig

# ================================
# 4. Clone PX4 & Setup Workspace
# ================================
# Cloned into /opt to prevent volume shadowing by Codespaces
WORKDIR /opt
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Run the PX4 setup script (specifically for simulation)
WORKDIR /opt/PX4-Autopilot
RUN ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# ================================
# 5. Environment Configuration
# ================================
# Setup rosdep
RUN rosdep init || true && rosdep update

# Auto-source ROS 2 
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# ================================
# 6. Build PX4 ROS 2 Messages (Underlay Workspace)
# ================================
WORKDIR /opt/px4_ros_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git \
    && git clone https://github.com/PX4/px4_ros_com.git

WORKDIR /opt/px4_ros_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
RUN echo "source /opt/px4_ros_ws/install/setup.bash" >> ~/.bashrc

# Ensure the terminal starts in the mounted Codespace directory
WORKDIR /workspaces
CMD ["/bin/bash"]