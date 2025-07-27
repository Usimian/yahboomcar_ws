# Navigation Guide for slam_nav Package

This guide explains how to use the comprehensive Nav2 navigation system integrated in the slam_nav package for the yahboomcar X3 robot.

## Navigation Components

The slam_nav package includes a complete Nav2 stack with the following components:

### Core Navigation Nodes

1. **bt_navigator**: Behavior tree navigator that coordinates navigation tasks
2. **controller_server**: Local trajectory controller using DWB (Dynamic Window Behavior)
3. **planner_server**: Global path planner using NavFn planner
4. **behavior_server**: Recovery behaviors (spin, backup, drive_on_heading, etc.)
5. **smoother_server**: Path smoothing for better trajectory quality
6. **velocity_smoother**: Velocity smoothing for smooth robot motion

### Supporting Nodes

1. **map_server**: Serves static maps for localization
2. **lifecycle_manager**: Manages node lifecycles
3. **waypoint_follower**: Sequential waypoint navigation
4. **collision_monitor**: Safety monitoring (if configured)

## Launch Files

### 1. SLAM + Navigation (`slam_nav.launch.py`)

Use this for simultaneous mapping and navigation:

```bash
ros2 launch slam_nav slam_nav.launch.py
```

**Key features:**
- Simultaneous SLAM mapping with slam_toolbox
- Full Nav2 navigation stack
- Real-time map building and navigation
- RViz2 visualization with navigation tools

### 2. Localization + Navigation (`localization_nav.launch.py`)

Use this when you have an existing map:

```bash
ros2 launch slam_nav localization_nav.launch.py map:=/path/to/your/map.yaml
```

**Key features:**
- AMCL localization with existing map
- Full Nav2 navigation stack
- Precise localization on known maps
- RViz2 visualization with navigation tools

## Using Navigation in RViz2

The RViz2 configuration includes all necessary tools for navigation:

### Setting Initial Pose

1. **Tool**: "2D Pose Estimate" (SetInitialPose)
2. **Usage**: 
   - Click and drag to set robot's initial position and orientation
   - Essential for AMCL localization
   - Published to `/initialpose` topic

### Setting Navigation Goals

1. **Tool**: "2D Goal Pose" (SetGoal)
2. **Usage**:
   - Click and drag to set desired robot position and orientation
   - Robot will automatically navigate to the goal
   - Published to `/goal_pose` topic

### Monitoring Navigation

- **Map Display**: Shows the occupancy grid map
- **Robot Model**: Shows robot's current pose
- **LaserScan**: Shows current lidar readings
- **Costmaps**: Global and local costmaps (can be enabled)
- **Path**: Shows planned global path (can be enabled)

## Navigation Configuration

### Robot Parameters (nav2_params.yaml)

Key parameters optimized for yahboomcar X3:

```yaml
# Robot physical constraints
robot_radius: 0.15                    # Robot's radius in meters
max_vel_x: 0.2                       # Maximum linear velocity
max_vel_theta: 0.5                   # Maximum angular velocity
acc_lim_x: 2.5                       # Linear acceleration limit
acc_lim_theta: 3.2                   # Angular acceleration limit

# Goal tolerances
xy_goal_tolerance: 0.25              # Position tolerance (25cm)
yaw_goal_tolerance: 0.25             # Orientation tolerance (~14 degrees)

# Costmap settings
update_frequency: 5.0                # Local costmap update rate
global_frame: map                    # Global coordinate frame
robot_base_frame: base_footprint     # Robot's base frame
```

### Behavior Tree Configuration

The navigation uses default Nav2 behavior trees:
- `navigate_to_pose_w_replanning_and_recovery.xml`
- `navigate_through_poses_w_replanning_and_recovery.xml`

These provide:
- Automatic replanning when path becomes invalid
- Recovery behaviors when robot gets stuck
- Goal cancellation and preemption

## Command Line Navigation

### Using Simple Commander API

```python
#!/usr/bin/env python3
import rclpy
from nav2_simple_commander import BasicNavigator

rclpy.init()
navigator = BasicNavigator()

# Set initial pose
initial_pose = # ... create PoseStamped
navigator.setInitialPose(initial_pose)

# Navigate to pose
goal_pose = # ... create PoseStamped
navigator.goToPose(goal_pose)

# Wait for navigation to complete
while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    # Process feedback

result = navigator.getResult()
```

### Using Action Client

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

# Create action client
nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

# Send goal
goal_msg = NavigateToPose.Goal()
goal_msg.pose = # ... create PoseStamped
future = nav_client.send_goal_async(goal_msg)
```

## Testing Navigation

Use the provided test script to verify navigation functionality:

```bash
# Make sure navigation is running first
ros2 launch slam_nav slam_nav.launch.py

# In another terminal, run the test
ros2 run slam_nav test_nav.py
```

The test will verify:
- Map availability
- Navigation action server
- Goal acceptance and cancellation

## Troubleshooting

### Common Issues

1. **Robot doesn't move**:
   - Check if initial pose is set correctly
   - Verify `/cmd_vel` topic is connected to robot base
   - Check costmap for obstacles

2. **Navigation fails frequently**:
   - Increase goal tolerances
   - Check robot's physical constraints match configuration
   - Verify sensor data quality

3. **Robot oscillates**:
   - Tune DWB controller parameters
   - Check `Oscillation` critic weight
   - Verify odometry quality

4. **Path planning fails**:
   - Check if goal is in free space
   - Verify map quality and resolution
   - Check planner tolerance settings

### Debugging Tools

```bash
# Check navigation status
ros2 topic echo /local_costmap/costmap_updates
ros2 topic echo /global_costmap/costmap_updates

# Monitor navigation feedback
ros2 topic echo /navigate_to_pose/_action/feedback

# Check behavior tree status
ros2 topic echo /behavior_tree_log
```

## Advanced Usage

### Waypoint Navigation

```python
# Navigate through multiple waypoints
waypoints = [pose1, pose2, pose3]
navigator.followWaypoints(waypoints)
```

### Custom Recovery Behaviors

The behavior server includes:
- **spin**: Rotate in place to clear costmap
- **backup**: Move backward to escape tight spaces  
- **drive_on_heading**: Move forward on specified heading
- **wait**: Pause navigation temporarily

### Performance Tuning

Key parameters to tune for your specific environment:

1. **Controller frequency**: Balance between responsiveness and CPU usage
2. **Costmap resolution**: Balance between accuracy and performance
3. **Planner tolerance**: Balance between success rate and accuracy
4. **Velocity limits**: Match robot's physical capabilities

## Safety Considerations

- Always test in a safe, controlled environment first
- Monitor robot behavior and be ready to stop manually
- Ensure emergency stop functionality is available
- Check sensor calibration regularly
- Verify obstacle detection is working properly

## Integration with Other Systems

The navigation system can be integrated with:
- **Vision systems**: For semantic navigation
- **Voice control**: For voice-commanded navigation
- **Web interfaces**: For remote navigation control
- **Multi-robot coordination**: For fleet management

This comprehensive navigation system provides robust autonomous navigation capabilities for the yahboomcar X3 robot while maintaining flexibility for customization and extension. 