# Robot Calibration Configuration

This directory contains the calibration parameters for the Yahboom X3 robot system.

## Files

### `robot_calibration.yaml`
Main calibration parameter file containing:

- **Hardware Driver Calibration Factors**: Applied to cmd_vel commands before sending to firmware
- **Odometry Scale Factors**: Applied in odometry calculations for position tracking  
- **Velocity Corrections**: Compensate for hardware under-reporting of actual velocities
- **Movement Limits**: Safety limits for velocities and accelerations
- **Physical Constants**: Robot dimensions and wheel parameters

## Usage

### In Launch Files
```python
calibration_config = os.path.join(              
    get_package_share_directory('yahboomcar_bringup'),
    'config',
    'robot_calibration.yaml'
)

node = Node(
    package='your_package',
    executable='your_node',
    parameters=[calibration_config]
)
```

### In Python Nodes
```python
# Declare parameters
self.declare_parameter('hardware_calibration.linear_x_cal_factor', 1.0)

# Read parameters  
linear_x_cal = self.get_parameter('hardware_calibration.linear_x_cal_factor').get_parameter_value().double_value
```

## Calibration Process

1. **Run Calibration Tests**: Use `robot_exercise` or `calibration_test` programs
2. **Measure Actual vs Expected Movement**: Compare commanded vs actual distances/angles
3. **Calculate Scale Factors**: `scale_factor = expected_distance / actual_distance`
4. **Update Parameters**: Modify values in `robot_calibration.yaml`
5. **Test and Iterate**: Repeat until movements are accurate

## Key Parameters Explained

### Hardware Calibration Factors
- `linear_x_cal_factor`: Multiplies forward/backward cmd_vel commands
- `linear_y_cal_factor`: Multiplies left/right strafe cmd_vel commands  
- `angular_cal_factor`: Multiplies rotational cmd_vel commands

### Velocity Corrections
- `linear_velocity_correction`: 1.52x correction for hardware velocity under-reporting
- Based on measurements: 0.62m actual / 0.408m reported = 1.52

### Odometry Scaling
- `linear_scale_x/y`: Scale factors for odometry position calculations
- `angular_scale`: Scale factor for odometry heading calculations

## Commands

### Run Robot Exercise with Calibration
```bash
ros2 launch yahboomcar_bringup robot_exercise_launch.py
```

### Run Main System with Calibration  
```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

### View Current Parameters
```bash
ros2 param list /Mcnamu_driver_X3
ros2 param get /Mcnamu_driver_X3 hardware_calibration.linear_x_cal_factor
```

### Set Parameters at Runtime
```bash
ros2 param set /Mcnamu_driver_X3 hardware_calibration.linear_x_cal_factor 1.1
```
