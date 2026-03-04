# ================================
# Base OS: Ubuntu 22.04 (Jammy)
# ================================
FROM ubuntu:22.04

# Set environment variables to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# ================================
# --- PHASE 1: ROOT (SYSTEM LEVEL) ---
# ================================
# Create developer user, grant passwordless sudo, and hand over /opt
RUN apt-get update && apt-get install -y sudo curl wget git lsb-release gnupg2 software-properties-common \
    && useradd -m -s /bin/bash -G sudo -u 1000 developer \
    && echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && chown -R developer:developer /opt

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# Install ALL apt dependencies (ROS 2, Gazebo, PX4 prereqs) in one clean sweep
RUN apt-get update && apt-get install -y \
    build-essential cmake ninja-build net-tools iputils-ping git-lfs \
    ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-ros-gz \
    python3-colcon-common-extensions python3-vcstool python3-rosdep \
    gazebo libgazebo-dev python3-pip python3-jinja2 python3-numpy \
    python3-empy python3-toml python3-packaging exiftool genromfs \
    libeigen3-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev astyle \
    && rm -rf /var/lib/apt/lists/*


# ================================
# --- PHASE 2: DEVELOPER (USER LEVEL) ---
# ================================
# Switch to the new user. Everything below this line is executed securely as 'developer'
USER developer
ENV PATH="/home/developer/.local/bin:$PATH"

RUN git lfs install

# 1. Build uXRCE-DDS Agent (Built by you, installed to system with sudo)
WORKDIR /opt
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent && mkdir build && cd build \
    && cmake .. && make \
    && sudo make install && sudo ldconfig

# 2. Clone PX4
WORKDIR /opt
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# 3. Run the PX4 setup script
WORKDIR /opt/PX4-Autopilot
# ubuntu.sh will use sudo for apt (which we allowed), and pip for your ~/.local folder
RUN ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# 4. Explicitly install python dependencies specifically for matplotlib version issues (this line was specific to the project at the time, can maybe remove later)
RUN pip3 install --user --force-reinstall "numpy<2" pyros-genmsg kconfiglib jsonschema jinja2 future cerberus pyyaml

# 5. Environment Configuration
RUN sudo rosdep init || true && rosdep update
RUN sudo bash -c 'echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc'

# 6. Build PX4 ROS 2 Messages
WORKDIR /opt/px4_ros_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git \
    && git clone https://github.com/PX4/px4_ros_com.git

WORKDIR /opt/px4_ros_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
RUN sudo bash -c 'echo "source /opt/px4_ros_ws/install/setup.bash" >> /etc/bash.bashrc'

# 7. Auto-source local workspace
RUN echo 'if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi' >> /home/developer/.bashrc

# Ensure the terminal starts in the mounted Codespace directory
WORKDIR /workspaces
CMD ["/bin/bash"]