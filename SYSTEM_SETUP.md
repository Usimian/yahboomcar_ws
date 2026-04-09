# System Setup Notes

These are system-level configuration changes made outside the ROS workspace
that are required for correct operation. They must be reproduced on a new system.

## CycloneDDS Unicast Configuration

CycloneDDS defaults to multicast discovery which floods the entire LAN with
traffic. We use unicast-only with explicit peer addresses instead.

### 1. Create the config file

Create `~/cyclonedds_unicast.xml`:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true" priority="default" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <EnableMulticastLoopback>false</EnableMulticastLoopback>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="10.0.0.119"/>
        <Peer address="127.0.0.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Replace `10.0.0.119` with the robot IP and add the workstation IP if different.

### 2. Add to ~/.bashrc

```bash
export CYCLONEDDS_URI=~/cyclonedds_unicast.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
```

The workstation side uses the same config, baked into the Docker image via
`llm-robot-ros/config/cyclonedds_unicast.xml`.

## PiOLED Boot Display

Displays IP address on the Adafruit PiOLED (SSD1306 128x32) at boot so you
know when the system is ready and what IP it has.

### 1. Install dependencies

```bash
pip3 install adafruit-circuitpython-ssd1306 adafruit-blinka Pillow
```

### 2. Create the script

Create `~/pioled_ws/boot_display.py` (see file in repo for content).
Make it executable: `chmod +x ~/pioled_ws/boot_display.py`

### 3. Create systemd service

Create `/etc/systemd/system/boot_display.service`:

```ini
[Unit]
Description=Boot display on PiOLED
After=network.target

[Service]
Type=oneshot
User=mw
Group=mw
SupplementaryGroups=gpio i2c
ExecStart=/home/mw/pioled_ws/boot_display.py
WorkingDirectory=/home/mw/pioled_ws
Environment="BLINKA_FORCEBOARD=JETSON_ORIN_NANO"
Environment="JETSON_MODEL_NAME=JETSON_ORIN_NANO"
Environment="HOME=/home/mw"
Environment="USER=mw"
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

Enable it: `sudo systemctl enable boot_display.service`

Note: `dev-i2c-1.device` dependency was removed — it uses the wrong path
format for this Jetson and prevents the service from starting.
