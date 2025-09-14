# Robot Exercise Program Usage

The Robot Exercise program is an interactive calibration tool for the Yahboom X3 robot.

## Prerequisites

The robot hardware must be running before starting the exercise program.

## Usage

### Step 1: Start Robot Hardware
In Terminal 1:
```bash
cd /home/mw/yahboomcar_ros2/yahboomcar_ws
source install/setup.bash
ros2 launch yahboomcar_bringup robot_bringup_launch.py
```

### Step 2: Start Robot Exercise Program
In Terminal 2:
```bash
cd /home/mw/yahboomcar_ros2/yahboomcar_ws
source install/setup.bash
ros2 run yahboomcar_bringup robot_exercise --ros-args --params-file src/yahboomcar_bringup/config/robot_calibration.yaml
```

## Commands

### Parameter Setting
- `lv <speed>` - Set linear velocity (m/s)
- `av <speed>` - Set angular velocity (deg/s)  
- `d <distance>` - Set target distance (m)
- `r <rotation>` - Set target rotation (deg)

### Movement Commands
- `move` - Move forward/backward (+ = forward, - = backward)
- `strafe` - Strafe left/right (+ = right, - = left)
- `turn` - Turn CW/CCW (+ = CW, - = CCW)

### Control Commands
- `stop` - Emergency stop
- `status` - Show current settings
- `help` - Show help
- `quit` - Exit program

## Notes

- The program loads calibration parameters from `robot_calibration.yaml`
- All movements use the calibrated parameters
- The program provides interactive feedback for calibration testing
- Use `quit` to exit the program properly
