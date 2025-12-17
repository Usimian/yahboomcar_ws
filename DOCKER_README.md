# Yahboomcar ROS2 Jazzy Docker Container

Containerized ROS2 Jazzy system for Yahboomcar X3 robot on Jetson Orin Nano.

## Quick Start

### 1. Prerequisites

- Jetson Orin Nano with JetPack R36.3+
- Docker with NVIDIA Container Runtime installed
- Udev rules configured (see UDEV_SETUP.md)
- ~10GB free disk space

### 2. Build the Container

```bash
cd ~/yahboomcar_ros2/yahboomcar_ws
./build_docker.sh
```

**Note:** First build takes 15-20 minutes on Jetson Orin Nano.

### 3. Run the Container

```bash
./run_docker.sh
```

The robot system will start automatically. Check logs:

```bash
docker compose logs -f
```

### 4. Access Container Shell

```bash
./shell_docker.sh
```

## What's Inside

- **Base:** Ubuntu 24.04 + ROS2 Jazzy
- **GPU:** NVIDIA CUDA 12.6 support
- **Hardware:** RealSense D435i, SLAMTEC S2 Lidar, Serial Motor Controller
- **Features:** SLAM Toolbox, Nav2, Robot Localization (EKF)

## Directory Structure

```
yahboomcar_ws/
├── Dockerfile              # Multi-stage Docker image definition
├── docker-compose.yml      # Service configuration
├── .dockerignore          # Files to exclude from build
├── docker/
│   ├── entrypoint.sh      # Main entrypoint script
│   └── ros_entrypoint.sh  # Alternative ROS entrypoint
├── config_overrides/      # Optional calibration overrides
├── maps/                  # Persistent SLAM maps
├── src/                   # ROS2 packages (your code)
├── build_docker.sh        # Build script
├── run_docker.sh          # Run script
└── shell_docker.sh        # Shell access script
```

## Common Operations

### View Logs

```bash
docker compose logs -f
```

### Stop Container

```bash
docker compose down
```

### Restart Container

```bash
docker compose restart
```

### Rebuild After Code Changes

```bash
./build_docker.sh
```

### Check Running Containers

```bash
docker ps
```

## Calibration Override

The container uses default calibration baked into the image. To override:

1. **Copy calibration files:**
   ```bash
   cp src/yahboomcar_bringup/config/robot_calibration.yaml config_overrides/
   cp src/yahboomcar_base_node/config/odometry_scaling.yaml config_overrides/
   ```

2. **Edit calibration:**
   ```bash
   nano config_overrides/robot_calibration.yaml
   ```

3. **Uncomment volume mounts in docker-compose.yml:**
   ```yaml
   volumes:
     - ./config_overrides/robot_calibration.yaml:/ros_ws/install/yahboomcar_bringup/share/yahboomcar_bringup/config/robot_calibration.yaml:ro
     - ./config_overrides/odometry_scaling.yaml:/ros_ws/install/yahboomcar_base_node/share/yahboomcar_base_node/config/odometry_scaling.yaml:ro
   ```

4. **Restart:**
   ```bash
   docker compose restart
   ```

## Map Persistence

Maps are automatically saved to `./maps/` on the host and persist across container restarts.

### Save a Map Manually

```bash
docker compose exec yahboomcar_ros2 \
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/ros_ws/maps/my_map'}}"
```

## Troubleshooting

### Container won't start

**Check logs:**
```bash
docker compose logs
```

**Common issues:**
- Device not found: Check `/dev/ttyUSB0`, `/dev/rplidar`, `/dev/video*`
- Permission denied: Check udev rules (see UDEV_SETUP.md)
- Out of memory: Increase memory limit in docker-compose.yml

### Camera not working

```bash
# Check camera detection
docker compose exec yahboomcar_ros2 rs-enumerate-devices

# Check USB connection
lsusb | grep Intel

# Check camera topics
docker compose exec yahboomcar_ros2 ros2 topic list | grep camera
```

### Lidar not working

```bash
# Check lidar device
ls -la /dev/rplidar

# If not found, check udev rules (see UDEV_SETUP.md)

# Check lidar topic
docker compose exec yahboomcar_ros2 ros2 topic echo /scan --once
```

### Motor controller not responding

```bash
# Check serial device
ls -la /dev/ttyUSB0

# Check permissions
docker compose exec yahboomcar_ros2 id
# Should show groups: rosuser dialout video plugdev

# Check velocity commands
docker compose exec yahboomcar_ros2 ros2 topic echo /vel_raw
```

### ROS2 nodes can't communicate

The container uses `network_mode: host` for ROS2 DDS discovery. Check:

```bash
# Verify network mode
docker inspect yahboomcar_ros2 | grep NetworkMode

# Check ROS_DOMAIN_ID
docker compose exec yahboomcar_ros2 env | grep ROS_DOMAIN_ID
```

## Development Workflow

### Modifying Code

1. **Edit source files on host** (in `src/`)

2. **Rebuild container:**
   ```bash
   ./build_docker.sh
   ```

3. **Restart:**
   ```bash
   docker compose restart
   ```

### Testing Individual Nodes

```bash
# Access container shell
./shell_docker.sh

# Run individual nodes
ros2 run yahboomcar_bringup Mcnamu_driver_X3

# Launch specific components
ros2 launch yahboomcar_bringup robot_bringup_launch.py
```

### Debugging

```bash
# Interactive shell
./shell_docker.sh

# Check topics
ros2 topic list

# Echo topic
ros2 topic echo /scan

# Check nodes
ros2 node list

# Check node info
ros2 node info /slam_toolbox

# Check tf tree
ros2 run tf2_tools view_frames
```

## GPU Monitoring

```bash
# Check GPU usage
nvidia-smi

# Monitor in real-time
watch -n 1 nvidia-smi
```

## Backup and Restore

### Backup Configuration

```bash
tar -czf yahboomcar_backup_$(date +%Y%m%d).tar.gz \
  config_overrides/ \
  maps/ \
  docker-compose.yml
```

### Restore Configuration

```bash
tar -xzf yahboomcar_backup_YYYYMMDD.tar.gz
```

## Performance Tuning

### Reduce Memory Usage

In `docker-compose.yml`, adjust:
```yaml
deploy:
  resources:
    limits:
      memory: 2G  # Reduce from 4G
```

### Increase Performance

- Ensure USB 3.0 for RealSense camera
- Use CUDA acceleration (already enabled)
- Adjust SLAM resolution in `src/slam_nav/config/slam_toolbox_config.yaml`

## Further Reading

- **Detailed Plan:** See `CONTAINERIZATION_PLAN.md`
- **Udev Setup:** See `UDEV_SETUP.md`
- **ROS2 Jazzy Docs:** https://docs.ros.org/en/jazzy/
- **Docker Compose:** https://docs.docker.com/compose/

## Support

For issues:
1. Check troubleshooting section above
2. Review logs: `docker compose logs`
3. Check hardware connections
4. Verify udev rules

## License

Same as the Yahboomcar ROS2 workspace.
