# Yahboom X3 — host setup (state not in any ROS package)

Robot host configuration that the working system depends on but which lives in
`/etc`, `~/`, systemd, the MCU, and the servo — **not** in this colcon workspace's
packages. If the Jetson is reflashed, reapply this. (The custom STM32 firmware + the
servo baud are documented separately in the **`rosmaster-sts-fw`** repo / `STS_PATCH.md`.)

Robot: `mw@10.0.0.119` (Jetson Orin Nano, JetPack 7.2 / Ubuntu 24.04 / ROS 2 Jazzy).

> **Currently-lapsed items (verified 2026-06-08):** WiFi reg-domain is `country 00`
> (should be US) and `nicou` powersave is `default` (should be disabled). These weren't
> reapplied after the 2026-06-05 reflash — reapply if you see 5 GHz dropouts / RTT spikes.

---

## systemd services (all `enabled`)

| Unit | Purpose | Source of truth |
|---|---|---|
| `robot_hardware.service` | always-on hardware core (Mcnamu driver, base_node, joy, joy_node) → `robot_hardware_launch.sh` | `/home/mw/robot_hardware_launch.sh` |
| `camera_tilt.service` | camera-tilt bridge (`/camera_tilt`→`/bus_servo`) | unit committed in `src/camera_tilt_bridge/systemd/camera_tilt.service` |
| `boot_display.service` | PiOLED status display (beeps 3× when battery service answers = "ready") | `/home/mw/pioled_ws/` |

Install/enable a unit: `sudo cp <unit> /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now <unit>`

**`boot_display.service` gotcha (fixed 2026-06-08):** it must NOT depend on
`dev-i2c-7.device` — i2c char devices never get the systemd udev tag, so that dep hangs
for the full 90 s device timeout (the "2 minutes to beep" bug). Use `ConditionPathExists=/dev/i2c-7`
instead, and `After=network.target robot_hardware.service`.

---

## Clock sync — chrony (required, or Nav2 throws TF/timestamp errors)

`/etc/chrony/chrony.conf` (robot = client of the workstation):
```
server 10.0.0.152 iburst minpoll 4 maxpoll 6
driftfile /var/lib/chrony/chrony.drift
makestep 1.0 3
rtcsync
logdir /var/log/chrony
```
`sudo systemctl disable --now systemd-timesyncd` (so it doesn't fight chrony).
The **workstation** runs the chrony *server* (`allow 10.0.0.0/24`, `port 123`).

## UDP buffers — `/etc/sysctl.d/60-ros2-buffers.conf` (both machines)
```
net.core.rmem_max=8388608
net.core.rmem_default=8388608
net.core.wmem_max=8388608
net.core.wmem_default=8388608
```
`sudo sysctl --system` to apply. (Without this, high-rate sensor topics drop under CycloneDDS.)

## WiFi (must be 5 GHz + powersave off, or 100–400 ms RTT spikes look like Nav2 faults)
```
# disable power save on the WiFi connection (name = "nicou")
sudo nmcli connection modify nicou 802-11-wireless.powersave 2
# force US reg domain so 5 GHz isn't passive-scan-only:
echo 'options cfg80211 ieee80211_regdom=US' | sudo tee /etc/modprobe.d/cfg80211.conf
# verify: `iw reg get` -> country US ; stay on the 5 GHz SSID (2.4 GHz = 100–170 ms RTT)
```

## sudoers — RealSense watchdog re-authorize
`/etc/sudoers.d/realsense-watchdog`:
```
mw ALL=(root) NOPASSWD: /usr/bin/tee /sys/bus/usb/devices/2-1.3/authorized
```
(`2-1.3` is the D435i's USB path. The `realsense_watchdog` node lives in the `slam_nav`
package — committed — and re-authorizes the camera if it drops.)

---

## Python / PyTorch (on-robot perception — optional, works)

`/home/mw/torch-env` venv: **PyTorch runs on JP7.2** via the official `cu130` aarch64 wheel
(sm_80 cubins are binary-compatible with the Orin's sm_87). Recreate:
```
sudo apt-get install -y python3-venv
python3 -m venv ~/torch-env && source ~/torch-env/bin/activate && pip install -U pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130   # MUST be cu130
pip install ultralytics                                                            # after torch
```
Benchmark (Orin Nano @25W): YOLO12m @640 = 11 FPS fp32 / 22 FPS fp16.

## Other installed bits
- apt: `stm32flash` (MCU flashing), `python3-venv`
- `~/.local`: `feetech-servo-sdk` (`pip3 install --user --break-system-packages feetech-servo-sdk`) — for setting servo baud over the USB board

---

## Custom MCU firmware + servo (see `rosmaster-sts-fw` repo)

- The expansion board runs **patched firmware** so its bus port drives the STS3215
  (little-endian STS write). Flash + recovery procedure: `rosmaster-sts-fw/STS_PATCH.md`.
- The **STS3215 tilt servo is set to 115200 baud** (to match the board's bus).
- Recovery hexes also kept on the robot at `/home/mw/rosmaster_V3.5.1{,_STS}.hex`.

---

## Fresh-robot reapply checklist
1. chrony.conf + disable timesyncd; sysctl drop-in (`sysctl --system`)
2. WiFi: powersave off + regdom US; join the 5 GHz SSID
3. sudoers realsense-watchdog drop-in
4. `colcon build` the workspace; `enable --now` the three services
5. flash the patched MCU firmware + set servo baud=115200 (`rosmaster-sts-fw`)
6. (optional) recreate `torch-env` for on-robot perception
