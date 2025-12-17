# Udev Rules Setup for Yahboomcar Docker Container

The Docker container needs access to hardware devices. Some devices require udev rules to create consistent symlinks.

## Required Udev Rules

Create the following file on the **host system** (not in container):

### File: `/etc/udev/rules.d/99-yahboomcar.rules`

```bash
# SLAMTEC Lidar - Create /dev/rplidar symlink
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="rplidar"

# Yahboom Motor Controller (backup, in case different USB-Serial adapter)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"

# Intel RealSense Camera (if needed)
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE="0666", GROUP="video"
```

## Installation Steps

### 1. Create the udev rules file

```bash
sudo nano /etc/udev/rules.d/99-yahboomcar.rules
```

Paste the rules above, then save and exit (Ctrl+X, Y, Enter).

### 2. Reload udev rules

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. Verify the symlink

After plugging in the lidar:

```bash
ls -la /dev/rplidar
ls -la /dev/ttyUSB*
```

You should see:
- `/dev/rplidar` -> `/dev/ttyUSB*` (symlink)
- `/dev/ttyUSB0` or `/dev/ttyUSB1` (actual devices)

### 4. Check device permissions

```bash
groups $USER
```

Your user should be in the `dialout` and `video` groups. If not:

```bash
sudo usermod -aG dialout,video $USER
```

Then log out and log back in for the changes to take effect.

## Troubleshooting

### Device not found

If `/dev/rplidar` doesn't appear:

1. **Check if device is connected:**
   ```bash
   lsusb | grep -i "Silicon Labs\|10c4"
   ```

2. **Check vendor/product IDs:**
   ```bash
   udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct"
   ```

3. **Manually test the rule:**
   ```bash
   sudo udevadm test /sys/class/tty/ttyUSB0
   ```

### Permission denied

If you get permission errors in the container:

1. **Check host permissions:**
   ```bash
   ls -la /dev/rplidar /dev/ttyUSB*
   ```

   Should show `crw-rw-rw-` (mode 0666).

2. **Verify group membership:**
   ```bash
   id -Gn
   ```

   Should include `dialout` and `video`.

3. **Restart container after fixing permissions:**
   ```bash
   docker compose restart
   ```

## Alternative: Finding Device IDs

If the udev rules don't work, you may need to find the exact vendor/product IDs for your devices:

```bash
# List all USB devices with details
lsusb -v

# Find serial device info
udevadm info -a /dev/ttyUSB0

# Find video device info
udevadm info -a /dev/video0
```

Then update the udev rules with the correct IDs.

## Testing in Container

After setting up udev rules, test device access in the container:

```bash
# Start container
./run_docker.sh

# Open shell in container
./shell_docker.sh

# Inside container, check devices:
ls -la /dev/ttyUSB0 /dev/rplidar /dev/video*

# Test lidar connection
rs-enumerate-devices  # For RealSense camera
```

If devices are accessible, you're ready to launch the robot!
