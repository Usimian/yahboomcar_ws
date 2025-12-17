# =======================================================================================
# Yahboomcar ROS2 Jazzy Containerization for Jetson Orin Nano
# =======================================================================================
# Base: NVIDIA L4T with CUDA 12.6 support
# Target: Ubuntu 24.04 + ROS2 Jazzy
# Hardware: Jetson Orin Nano with RealSense D435i, SLAMTEC S2 Lidar, Serial Motor Controller
# =======================================================================================

# ============================================
# Stage 1: Builder
# ============================================
FROM nvcr.io/nvidia/l4t-jetpack:r36.3.0 as builder

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# Install ROS2 Jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install librealsense2 (required for RealSense camera)
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -y --no-install-recommends \
        librealsense2-dev \
        librealsense2-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    setuptools \
    tf-transformations \
    Rosmaster-Lib==3.3.9 \
    pyserial \
    numpy \
    opencv-python \
    scipy

# Initialize rosdep
RUN rosdep init || true \
    && rosdep update --rosdistro ${ROS_DISTRO}

# Create workspace
WORKDIR /ros_ws
COPY src/ /ros_ws/src/

# Install dependencies using rosdep
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ============================================
# Stage 2: Runtime
# ============================================
FROM nvcr.io/nvidia/l4t-jetpack:r36.3.0

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV RCUTILS_COLORIZED_OUTPUT=1
ENV RCUTILS_LOGGING_USE_STDOUT=1
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-cv-bridge \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install librealsense2 runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -y --no-install-recommends \
        librealsense2 \
        librealsense2-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python runtime dependencies
RUN pip3 install --no-cache-dir \
    tf-transformations \
    Rosmaster-Lib==3.3.9 \
    pyserial \
    numpy \
    opencv-python \
    scipy

# Copy built workspace from builder
COPY --from=builder /ros_ws/install /ros_ws/install
COPY --from=builder /ros_ws/src /ros_ws/src

# Create user with same UID/GID as host for device access
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd -g ${USER_GID} rosuser && \
    useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash rosuser && \
    usermod -aG dialout,video,plugdev rosuser

# Create directories for volume mounts
RUN mkdir -p /ros_ws/config_overrides /ros_ws/maps && \
    chown -R rosuser:rosuser /ros_ws

# Copy entrypoint scripts
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /entrypoint.sh /ros_entrypoint.sh

# Switch to non-root user
USER rosuser
WORKDIR /ros_ws

# Source ROS2 workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROBOT_TYPE=X3" >> ~/.bashrc && \
    echo "export RPLIDAR_TYPE=s2" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "slam_nav", "robot_slam_nav_launch.py"]
