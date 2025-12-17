# Yahboomcar ROS2 Containerization Plan
## Migration from Native Ubuntu 22.04 + ROS2 Humble to Docker Container with Ubuntu 24.04 + ROS2 Jazzy

---

## Executive Summary

**Goal:** Containerize the Yahboomcar ROS2 workspace to run in Docker on Jetson Orin Nano with Ubuntu 24.04 and ROS2 Jazzy.

**Current State:**
- Native Ubuntu 22.04 with ROS2 Humble
- 9 ROS2 packages in `~/yahboomcar_ros2/yahboomcar_ws`
- Hardware: Jetson Orin Nano, RealSense D435i, SLAMTEC S2 Lidar, Serial motor controller

**Target State:**
- Docker container with Ubuntu 24.04 + ROS2 Jazzy
- NVIDIA GPU support for camera acceleration
- Hybrid calibration (defaults baked, volume override)
- Persistent map storage
- Docker Compose deployment

---

## Phase 1: Pre-Migration Code Fixes

Before building the container, we need to fix ROS2 Humble → Jazzy compatibility issues in the source code.

### 1.1 Fix tf_transformations Import (CRITICAL)

**File:** `src/slam_nav/slam_nav/initial_pose_publisher.py` (line 11)

**Issue:** `tf_transformations` package was removed in ROS2 Jazzy.

**Solution:** Install `tf-transformations` from PyPI (it's a standalone Python package now).

**Change Required:**
```python
# No code change needed in Python files
# Just add to Dockerfile: pip install tf-transformations
```

**Also affects:** `src/realsense-ros/realsense2_camera/scripts/set_cams_transforms.py` (line 26)
- This is example code, not used by main system, but should be fixed for completeness.

### 1.2 Remove Hardcoded Humble Path (CRITICAL)

**File:** `src/yahboomcar_base_node/CMakeLists.txt` (line 6)

**Current:**
```cmake
include_directories(
  ${turtlesim_INCLUDE_DIRS}
  /opt/ros/humble/include  # HARDCODED PATH
)
```

**Fix:**
```cmake
include_directories(
  ${turtlesim_INCLUDE_DIRS}
  # Removed hardcoded path - use find_package instead
)
```

### 1.3 Update CMakeLists.txt Version Requirements

**Files to update:**
- `src/yahboomcar_base_node/CMakeLists.txt`
- `src/robot_msgs/CMakeLists.txt`
- `src/realsense-ros/realsense2_camera/CMakeLists.txt`
- `src/realsense-ros/realsense2_camera_msgs/CMakeLists.txt`
- `src/realsense-ros/realsense2_description/CMakeLists.txt`

**Change:**
```cmake
# OLD:
cmake_minimum_required(VERSION 3.5)

# NEW:
cmake_minimum_required(VERSION 3.16)  # Jazzy requirement
```

**And set C++ standard:**
```cmake
# Add after project():
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
```

### 1.4 Update Python setup.py Files

**Files to update:**
- `src/yahboomcar_ctrl/setup.py`
- `src/yahboomcar_bringup/setup.py`
- `src/yahboomcar_description/setup.py`
- `src/slam_nav/setup.py`

**Add Python version requirement:**
```python
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # ... existing config ...
    python_requires='>=3.12',  # ADD THIS LINE
    # ... rest of config ...
)
```

---

## Phase 2: Directory Structure Setup

Create the following structure in your workspace:

```
yahboomcar_ws/
├── Dockerfile
├── docker-compose.yml
├── .dockerignore
├── docker/
│   ├── entrypoint.sh
│   └── ros_entrypoint.sh
├── config_overrides/          # For volume-mounted calibration overrides
│   ├── robot_calibration.yaml (optional)
│   └── odometry_scaling.yaml (optional)
├── maps/                       # For persistent map storage
└── src/                        # Existing source code
```

---

## Phase 3: Dockerfile Creation

### 3.1 Dockerfile Strategy

**Base Image:** Use NVIDIA L4T (Linux for Tegra) with CUDA support
- `nvcr.io/nvidia/l4t-jetpack:r36.3.0` (includes CUDA 12.6 for Jetson Orin)

**Multi-stage Build:**
- Stage 1: Build dependencies and compile ROS2 packages
- Stage 2: Runtime image with only necessary binaries

### 3.2 Dockerfile Content

**File:** `yahboomcar_ws/Dockerfile`

```dockerfile
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

# Apply code fixes before building
# Fix hardcoded Humble path
RUN sed -i '/\/opt\/ros\/humble\/include/d' /ros_ws/src/yahboomcar_base_node/CMakeLists.txt

# Update CMake version requirements
RUN find /ros_ws/src -name "CMakeLists.txt" -type f -exec sed -i 's/cmake_minimum_required(VERSION 3\.5)/cmake_minimum_required(VERSION 3.16)/g' {} \;

# Add C++17 standard to CMakeLists.txt files
RUN for file in $(find /ros_ws/src -name "CMakeLists.txt" -type f); do \
    if grep -q "project(" "$file" && ! grep -q "CMAKE_CXX_STANDARD" "$file"; then \
        sed -i '/project(/a\if(NOT CMAKE_CXX_STANDARD)\n  set(CMAKE_CXX_STANDARD 17)\nendif()' "$file"; \
    fi; \
done

# Install dependencies
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
```

### 3.3 .dockerignore File

**File:** `yahboomcar_ws/.dockerignore`

```
build/
install/
log/
.git/
.vscode/
*.pyc
__pycache__/
.cache/
*.swp
*.swo
*~
.DS_Store
config_overrides/
maps/
```

---

## Phase 4: Docker Compose Configuration

### 4.1 docker-compose.yml

**File:** `yahboomcar_ws/docker-compose.yml`

```yaml
version: '3.8'

services:
  yahboomcar_ros2:
    image: yahboomcar_ros2:jazzy
    container_name: yahboomcar_ros2
    build:
      context: .
      dockerfile: Dockerfile
      args:
        USER_UID: 1000
        USER_GID: 1000

    # NVIDIA GPU support
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - DISPLAY=${DISPLAY}

      # ROS2 configuration
      - ROS_DOMAIN_ID=0
      - ROS_LOCALHOST_ONLY=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROBOT_TYPE=X3
      - RPLIDAR_TYPE=s2
      - JETSON_MODEL_NAME=JETSON_ORIN_NANO

      # CUDA paths
      - CUDA_HOME=/usr/local/cuda
      - LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH

    # Device access
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0        # Motor controller
      - /dev/rplidar:/dev/rplidar        # Lidar (must create udev rule on host)
      - /dev/video0:/dev/video0          # RealSense camera
      - /dev/video1:/dev/video1
      - /dev/video2:/dev/video2
      - /dev/video3:/dev/video3
      - /dev/video4:/dev/video4
      - /dev/video5:/dev/video5
      - /dev/bus/usb:/dev/bus/usb        # USB device access

    # Volume mounts
    volumes:
      # Persistent map storage
      - ./maps:/ros_ws/maps

      # Optional calibration overrides (comment out to use baked defaults)
      # - ./config_overrides/robot_calibration.yaml:/ros_ws/install/yahboomcar_bringup/share/yahboomcar_bringup/config/robot_calibration.yaml:ro
      # - ./config_overrides/odometry_scaling.yaml:/ros_ws/install/yahboomcar_base_node/share/yahboomcar_base_node/config/odometry_scaling.yaml:ro

      # X11 for GUI applications (optional)
      - /tmp/.X11-unix:/tmp/.X11-unix:ro
      - ~/.Xauthority:/home/rosuser/.Xauthority:ro

    # Network mode - host required for ROS2 DDS discovery
    network_mode: host

    # Privileged mode for hardware access
    privileged: true

    # Restart policy
    restart: unless-stopped

    # Resource limits (optional, adjust as needed)
    deploy:
      resources:
        limits:
          memory: 4G
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
```

---

## Phase 5: Entrypoint Scripts

### 5.1 Main Entrypoint

**File:** `yahboomcar_ws/docker/entrypoint.sh`

```bash
#!/bin/bash
set -e

# Source ROS2 workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros_ws/install/setup.bash

# Check for calibration overrides
if [ -f "/ros_ws/config_overrides/robot_calibration.yaml" ]; then
    echo "Using calibration override from volume mount"
fi

# Print environment info
echo "======================================"
echo "Yahboomcar ROS2 Container"
echo "======================================"
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "ROBOT_TYPE: ${ROBOT_TYPE}"
echo "RPLIDAR_TYPE: ${RPLIDAR_TYPE}"
echo "======================================"

# Check device availability
echo "Checking hardware devices..."
[ -e /dev/ttyUSB0 ] && echo "✓ Motor controller (/dev/ttyUSB0) found" || echo "✗ Motor controller not found"
[ -e /dev/rplidar ] && echo "✓ Lidar (/dev/rplidar) found" || echo "✗ Lidar not found (check udev rules)"
[ -e /dev/video0 ] && echo "✓ Camera (/dev/video0) found" || echo "✗ Camera not found"
echo "======================================"

# Execute the command
exec "$@"
```

### 5.2 ROS Entrypoint (Alternative)

**File:** `yahboomcar_ws/docker/ros_entrypoint.sh`

```bash
#!/bin/bash
set -e

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros_ws/install/setup.bash

exec "$@"
```

---

## Phase 6: Build and Deployment Scripts

### 6.1 Build Script

**File:** `yahboomcar_ws/build_docker.sh`

```bash
#!/bin/bash
set -e

echo "Building Yahboomcar ROS2 Docker image..."
echo "This may take 15-20 minutes on Jetson Orin Nano"

# Build the image
docker compose build

echo ""
echo "Build complete!"
echo "To run: docker compose up"
```

### 6.2 Run Script

**File:** `yahboomcar_ws/run_docker.sh`

```bash
#!/bin/bash
set -e

# Allow X11 connections (for RViz if needed)
xhost +local:docker

# Start the container
docker compose up -d

echo "Container started!"
echo "To view logs: docker compose logs -f"
echo "To stop: docker compose down"
```

### 6.3 Development Shell Script

**File:** `yahboomcar_ws/shell_docker.sh`

```bash
#!/bin/bash
set -e

# Open a bash shell in the running container
docker compose exec yahboomcar_ros2 bash
```

---

## Phase 7: Udev Rules for Device Symlinks

The container needs `/dev/rplidar` to exist. Create this udev rule on the **host system**:

**File:** `/etc/udev/rules.d/99-yahboomcar.rules` (on host)

```
# SLAMTEC Lidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="rplidar"

# Yahboom Motor Controller (backup, in case different device)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"
```

**Apply rules:**
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## Phase 8: Testing Strategy

### 8.1 Pre-Container Testing Checklist

Before building the container, test code fixes on native system:

```bash
# 1. Apply code fixes manually (as described in Phase 1)
# 2. Test build with fixed code
cd ~/yahboomcar_ros2/yahboomcar_ws
colcon build

# 3. Test launch
ros2 launch slam_nav robot_slam_nav_launch.py

# 4. Verify all topics
ros2 topic list

# 5. Check for errors
ros2 wtf
```

### 8.2 Container Build Testing

```bash
cd ~/yahboomcar_ros2/yahboomcar_ws

# Build image
./build_docker.sh

# Verify image created
docker images | grep yahboomcar
```

### 8.3 Container Runtime Testing

```bash
# Start container
./run_docker.sh

# Check logs for errors
docker compose logs -f

# In another terminal, verify ROS2 topics
docker compose exec yahboomcar_ros2 ros2 topic list

# Check device access
docker compose exec yahboomcar_ros2 bash
# Inside container:
ls -la /dev/ttyUSB0 /dev/rplidar /dev/video*
```

### 8.4 Hardware Validation

```bash
# Test motor controller
docker compose exec yahboomcar_ros2 ros2 topic echo /vel_raw

# Test lidar
docker compose exec yahboomcar_ros2 ros2 topic echo /scan

# Test camera
docker compose exec yahboomcar_ros2 ros2 topic echo /camera/color/image_raw --once

# Test SLAM
docker compose exec yahboomcar_ros2 ros2 topic echo /map --once
```

### 8.5 Performance Testing

```bash
# Check CPU/Memory usage
docker stats yahboomcar_ros2

# Check GPU usage (should show CUDA acceleration)
nvidia-smi

# Monitor ROS2 performance
docker compose exec yahboomcar_ros2 ros2 wtf
```

---

## Phase 9: Calibration Override Workflow

### Using Default Calibration (Baked into Image)

Just run normally - no extra steps needed:
```bash
docker compose up
```

### Overriding Calibration with Host Files

1. **Copy default calibration to host:**
```bash
# Create override directory
mkdir -p ~/yahboomcar_ros2/yahboomcar_ws/config_overrides

# Copy current calibration
cp ~/yahboomcar_ros2/yahboomcar_ws/src/yahboomcar_bringup/config/robot_calibration.yaml \
   ~/yahboomcar_ros2/yahboomcar_ws/config_overrides/

cp ~/yahboomcar_ros2/yahboomcar_ws/src/yahboomcar_base_node/config/odometry_scaling.yaml \
   ~/yahboomcar_ros2/yahboomcar_ws/config_overrides/
```

2. **Edit calibration files:**
```bash
nano ~/yahboomcar_ros2/yahboomcar_ws/config_overrides/robot_calibration.yaml
# Adjust calibration values...
```

3. **Uncomment volume mounts in docker-compose.yml:**
```yaml
# Uncomment these lines:
- ./config_overrides/robot_calibration.yaml:/ros_ws/install/yahboomcar_bringup/share/yahboomcar_bringup/config/robot_calibration.yaml:ro
- ./config_overrides/odometry_scaling.yaml:/ros_ws/install/yahboomcar_base_node/share/yahboomcar_base_node/config/odometry_scaling.yaml:ro
```

4. **Restart container:**
```bash
docker compose restart
```

---

## Phase 10: Map Persistence Workflow

### Enabling Map Saving

Maps are automatically persisted to `~/yahboomcar_ros2/yahboomcar_ws/maps/` on the host.

To enable map saving in SLAM Toolbox:

1. **Update SLAM config** (`src/slam_nav/config/slam_toolbox_config.yaml`):
```yaml
use_map_saver: true
map_file_name: /ros_ws/maps/my_map
```

2. **Rebuild and restart:**
```bash
docker compose build
docker compose restart
```

### Saving Maps Manually

```bash
# Inside container or from host:
docker compose exec yahboomcar_ros2 \
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/ros_ws/maps/my_map'}}"
```

### Loading Saved Maps

Maps saved to `/ros_ws/maps/` persist on the host and can be loaded on next run.

---

## Phase 11: Troubleshooting Guide

### Issue: Device not found

**Symptoms:** Container starts but hardware devices not accessible

**Solutions:**
1. Check udev rules: `ls -la /dev/rplidar /dev/ttyUSB*`
2. Verify permissions: User must be in `dialout` group
3. Verify device mapping in docker-compose.yml
4. Try privileged mode (already enabled)

### Issue: Camera not working

**Symptoms:** RealSense node fails to start

**Solutions:**
1. Check USB connection: `lsusb | grep Intel`
2. Verify librealsense: `docker compose exec yahboomcar_ros2 rs-enumerate-devices`
3. Check USB 3.0 connection (camera requires USB 3.0)
4. Rebuild with `--no-cache` if library mismatch

### Issue: Build fails with CMake errors

**Symptoms:** Dockerfile build fails during colcon build

**Solutions:**
1. Check code fixes were applied (Phase 1)
2. Verify internet connection for package downloads
3. Clear Docker cache: `docker system prune -a`
4. Check Jetson disk space: `df -h`

### Issue: ROS2 nodes can't communicate

**Symptoms:** Topics not visible between container and host

**Solutions:**
1. Verify network_mode: host in docker-compose.yml
2. Check ROS_DOMAIN_ID matches (should be 0)
3. Verify firewall not blocking DDS ports
4. Check `/etc/hosts` file for hostname resolution

### Issue: Container crashes immediately

**Symptoms:** Container exits with error code

**Solutions:**
1. Check logs: `docker compose logs`
2. Verify device paths exist before starting
3. Try interactive mode: `docker compose run yahboomcar_ros2 bash`
4. Check resource limits (memory)

---

## Phase 12: Maintenance and Updates

### Updating Source Code

After making changes to source code:

```bash
# Option 1: Rebuild entire image
docker compose build

# Option 2: Rebuild only changed packages (faster)
docker compose exec yahboomcar_ros2 bash
# Inside container:
cd /ros_ws
colcon build --packages-select <package_name>
source install/setup.bash
# Exit and restart:
exit
docker compose restart
```

### Updating Calibration

See Phase 9 for calibration override workflow.

### Updating ROS2 Packages

```bash
# Rebuild image to get latest packages
docker compose build --no-cache
```

### Backing Up Configuration

```bash
# Backup entire configuration
tar -czf yahboomcar_backup_$(date +%Y%m%d).tar.gz \
  ~/yahboomcar_ros2/yahboomcar_ws/config_overrides \
  ~/yahboomcar_ros2/yahboomcar_ws/maps \
  ~/yahboomcar_ros2/yahboomcar_ws/docker-compose.yml
```

---

## Phase 13: Implementation Checklist

### Pre-Implementation
- [ ] Read and understand entire plan
- [ ] Backup current working system
- [ ] Verify disk space (need ~10GB for image)
- [ ] Ensure internet connection stable

### Code Fixes (Phase 1)
- [ ] Fix tf_transformations imports
- [ ] Remove hardcoded /opt/ros/humble path
- [ ] Update CMakeLists.txt version requirements
- [ ] Update setup.py Python version requirements
- [ ] Test build on native system

### Docker Setup (Phases 2-5)
- [ ] Create directory structure
- [ ] Write Dockerfile
- [ ] Write .dockerignore
- [ ] Write docker-compose.yml
- [ ] Write entrypoint scripts
- [ ] Make scripts executable

### Host Configuration (Phase 7)
- [ ] Create udev rules
- [ ] Reload udev rules
- [ ] Verify /dev/rplidar exists

### Build & Test (Phase 8)
- [ ] Build Docker image
- [ ] Verify image created
- [ ] Start container
- [ ] Check logs for errors
- [ ] Verify device access
- [ ] Test all hardware
- [ ] Validate ROS2 topics

### Optional Configuration
- [ ] Set up calibration overrides (Phase 9)
- [ ] Configure map persistence (Phase 10)
- [ ] Test recovery from failures

### Documentation
- [ ] Document any custom changes
- [ ] Update README if exists
- [ ] Save this plan for future reference

---

## Success Criteria

The containerization is successful when:

1. ✅ Container builds without errors
2. ✅ All hardware devices accessible (/dev/ttyUSB0, /dev/rplidar, camera)
3. ✅ ROS2 nodes launch successfully
4. ✅ Robot can move via joystick control
5. ✅ Lidar publishes scan data
6. ✅ Camera publishes images and depth
7. ✅ SLAM creates map
8. ✅ EKF fuses odometry and IMU
9. ✅ Maps persist across container restarts
10. ✅ GPU acceleration working (check with nvidia-smi)

---

## Estimated Timeline

- **Code Fixes:** 30-60 minutes
- **Docker Setup:** 1-2 hours
- **First Build:** 15-20 minutes
- **Testing & Debugging:** 2-4 hours
- **Documentation:** 30 minutes

**Total:** 5-8 hours for complete migration

---

## Rollback Plan

If containerization fails:

1. **Native system is untouched** - original workspace still works
2. Remove container: `docker compose down`
3. Remove image: `docker rmi yahboomcar_ros2:jazzy`
4. Continue using native system
5. Revert code changes if applied

---

## Future Enhancements

After successful containerization:

1. **CI/CD Pipeline:** Automate image builds
2. **Multi-arch Support:** Build for x86_64 for simulation
3. **Remote Development:** VSCode devcontainer integration
4. **Orchestration:** Kubernetes/K3s for production
5. **Monitoring:** Prometheus/Grafana for metrics
6. **Logging:** Centralized logging with ELK stack

---

## References

- ROS2 Jazzy Documentation: https://docs.ros.org/en/jazzy/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/
- Docker Compose: https://docs.docker.com/compose/
- Intel RealSense ROS2: https://github.com/IntelRealSense/realsense-ros
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox

---

## Contact & Support

For issues specific to this containerization:
1. Check troubleshooting guide (Phase 11)
2. Review Docker logs: `docker compose logs`
3. Test components individually
4. Verify hardware connections

---

**End of Plan**
