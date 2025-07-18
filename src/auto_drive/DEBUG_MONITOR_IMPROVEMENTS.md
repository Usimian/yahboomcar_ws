# Debug Monitor Improvements

## Issues Fixed

### 1. **Sideways Movement Not Calculated**
**Problem**: The debug monitor was not showing sideways movement (linear.y) for mecanum wheel robots.

**Solution**: 
- Added proper display of all velocity components:
  - Forward/Backward movement (linear.x)
  - Sideways movement (linear.y) - important for mecanum wheels
  - Rotation (angular.z)
- Added total speed calculation: `sqrt(linear.x² + linear.y²)`
- Added movement direction calculation in degrees (0° = forward, 90° = left)

### 2. **Battery Level Monitoring Missing**
**Problem**: No battery level monitoring was available in the debug monitor.

**Solution**:
- Added subscription to `/voltage` topic
- Implemented battery status categorization:
  - **HIGH**: > 12.0V - Fully charged
  - **GOOD**: 11.0V - 12.0V - Normal operation
  - **LOW**: 10.0V - 11.0V - Should recharge soon
  - **CRITICAL**: < 10.0V - Recharge immediately

### 3. **Position Display Instability**
**Problem**: Position display was constantly changing even when robot was stopped due to odometry noise.

**Solution**:
- Added position filtering with 1cm threshold
- Only updates displayed position when robot moves significantly
- Provides stable position display when robot is stationary

### 4. **Incorrect Obstacle Distances**
**Problem**: Obstacle distance calculations were using wrong lidar coordinate system and angle ranges.

**Solution**:
- **Corrected lidar coordinate system**: 0° = forward, 90° = left, -90° = right
- **Fixed physical limitations**: S2 lidar only covers -120° to +120° (240° total)
- **Accounted for hardware blocking**: Back of robot blocked by CPU - no lidar coverage
- **Improved angle normalization**: Proper handling of -180° to +180° range
- **Added invalid reading filtering**: Skip NaN, infinity, and out-of-range readings

## Physical Hardware Limitations

### S2 Lidar Coverage
- **Coverage**: 240° total (-120° to +120°)
- **Blocked area**: Back of robot (CPU blocking)
- **Valid sectors**:
  - Front: -30° to +30°
  - Left: 60° to 120°
  - Right: -120° to -60°
  - Back: N/A (blocked by CPU)

## New Features

### Enhanced Velocity Display
```
Velocity Commands:
  Forward/Backward: 0.200 m/s
  Sideways (Left/Right): 0.300 m/s
  Rotation: 0.000 rad/s
  Total Speed: 0.361 m/s
  Direction: 56.3° (0°=forward, 90°=left)
```

### Battery Status Display
```
Battery: GOOD (11.45V)
```

### Improved Obstacle Detection
```
Obstacle Distances:
  Front: 2.500m
  Left:  1.800m
  Right: 3.200m
  Back:  N/A (blocked by CPU)
  Lidar Range: -120.0° to 120.0° (240° coverage)
```

### Stable Position Display
```
Position: X=1.500m, Y=2.300m  (filtered, stable when stopped)
```

## Usage

### Running the Debug Monitor
```bash
# Start the debug monitor
ros2 run auto_drive debug_monitor
```

### Testing the Debug Monitor
```bash
# Run test script to verify functionality
python3 src/auto_drive/test_debug_monitor.py
```

The test script will cycle through different scenarios:
1. Forward movement
2. Sideways movement (mecanum wheels)
3. Rotation
4. Different battery levels

## Technical Details

### New Subscriptions
- `/voltage` (Float32) - Battery voltage monitoring

### Enhanced Calculations
- **Total Speed**: `sqrt(linear.x² + linear.y²)`
- **Direction**: `atan2(linear.y, linear.x)` in degrees
- **Battery Status**: Voltage-based categorization
- **Position Filtering**: 1cm threshold for stability
- **Lidar Angle Normalization**: Proper -180° to +180° handling

### Corrected Lidar Processing
- **Angle Calculation**: `angle_rad * 180.0 / math.pi`
- **Angle Normalization**: Handle wrap-around at ±180°
- **Invalid Reading Filter**: Skip NaN, infinity, out-of-range
- **Physical Limitations**: Respect 240° coverage limit
- **Sector Definitions**: 
  - Front: -30° to +30°
  - Left: 60° to 120°
  - Right: -120° to -60°
  - Back: Not available (CPU blocked)

### Improved Display Format
- Wider display (70 characters)
- Better organized sections
- More detailed information
- Clear labeling of all components
- Hardware limitation indicators

## Benefits

1. **Complete Velocity Monitoring**: Now shows all movement components including sideways motion
2. **Battery Safety**: Warns about low battery conditions that could affect navigation
3. **Stable Position Display**: Filters out noise when robot is stopped
4. **Accurate Obstacle Detection**: Correct lidar coordinate system and physical limitations
5. **Better Debugging**: More detailed information helps diagnose navigation issues
6. **Mecanum Wheel Support**: Properly displays sideways movement capabilities
7. **Hardware Awareness**: Accounts for physical lidar limitations and CPU blocking
8. **Professional Display**: Clean, organized output format

## Integration

The improved debug monitor works seamlessly with:
- Autonomous navigation system
- Manual control modes
- Battery monitoring systems
- S2 lidar with 240° coverage
- Pose estimation systems
- Mecanum wheel control

This provides comprehensive real-time monitoring for the yahboomcar autonomous navigation system while respecting the physical hardware limitations of the S2 lidar sensor. 