# 🚀 KaCanOpen ROS 2 - Quick Start Guide

## 1. Basic Usage (Docker - Recommended)

```bash
# Clone and enter directory
git clone <your-repo>/kacanopen.git
cd kacanopen

# Start the system
docker compose up

# In another terminal, check topics
docker compose exec kacanopen bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"
```

## 2. Key ROS 2 Topics

### 📤 Published Topics (150Hz)
```bash
# Motor states (position, velocity, effort)
/device1/get_joint_state    # sensor_msgs/msg/JointState
/device3/get_joint_state    # sensor_msgs/msg/JointState  
/device4/get_joint_state    # sensor_msgs/msg/JointState
/device6/get_joint_state    # sensor_msgs/msg/JointState

# Motor status
/device1/profile_velocity_mode    # std_msgs/msg/Int32
/device3/profile_velocity_mode    # std_msgs/msg/Int32
/device4/profile_velocity_mode    # std_msgs/msg/Int32
/device6/profile_velocity_mode    # std_msgs/msg/Int32
```

### 📥 Subscribed Topics
```bash
# Send velocity commands
/device1/set_joint_state    # sensor_msgs/msg/JointState
/device3/set_joint_state    # sensor_msgs/msg/JointState
/device4/set_joint_state    # sensor_msgs/msg/JointState
/device6/set_joint_state    # sensor_msgs/msg/JointState
```

## 3. Common Commands

### Monitor Motor States
```bash
# Watch joint states at 150Hz
ros2 topic hz /device1/get_joint_state

# See actual values
ros2 topic echo /device1/get_joint_state
```

### Send Velocity Commands
```bash
# Send velocity command to motor 1 (e.g. 1000)
ros2 topic pub /device1/set_joint_state sensor_msgs/msg/JointState "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
name: ['motor1']
position: [0.0]
velocity: [1000.0]
effort: [0.0]
"

# Stop motor 1
ros2 topic pub /device1/set_joint_state sensor_msgs/msg/JointState "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
name: ['motor1']
position: [0.0]
velocity: [0.0]
effort: [0.0]
"
```

### Services
```bash
# Reset all motors (full CAN reset + re-initialization)
ros2 service call /reset_motors std_srvs/srv/Trigger "{}"
```

## 4. Configuration

### Environment Variables (.env)
```bash
# CAN settings
BUSNAME=can0
BAUDRATE=1M

# Motor parameters
ACCELERATION=10000
DECELERATION=20000

# ROS 2 settings
ROS_DOMAIN_ID=0
```

### Custom Launch
```bash
# Manual launch with parameters
ros2 launch kacanopen espeleo_kacanopen_launch.py \
    busname:=can0 \
    baudrate:=1M \
    acceleration:=10000
```

## 5. Troubleshooting

### Check CAN Interface
```bash
# Verify CAN is up
ip link show can0

# Bring up CAN interface
sudo ip link set can0 up type can bitrate 1000000

# Monitor CAN traffic
candump can0
```

### Debug Docker
```bash
# View logs
docker compose logs -f

# Enter container
docker compose exec kacanopen bash

# Restart service
docker compose restart
```

### Performance Issues
```bash
# Check topic frequency
ros2 topic hz /device1/get_joint_state

# Should show ~150Hz average rate
# If lower, check:
# - ipc: host in docker-compose.yml
# - /dev/shm volume mount
# - ROS_LOCALHOST_ONLY=1 environment
```

## 6. Hardware Setup

### CAN Wiring
- Ensure proper 120Ω termination on both ends
- Use twisted pair cable for CAN_H/CAN_L
- Common ground between all devices

### Motor Configuration
- Set correct node IDs (1, 3, 4, 6 by default)
- Configure baudrate to match (1Mbps default)
- Enable CANopen mode on motor controllers

## 7. Integration Examples

### With MoveIt2
```yaml
# robot_description.yaml
hardware:
  - name: kacanopen_system
    type: kacanopen/KaCanopenHardware
    parameters:
      busname: can0
      baudrate: 1M
```

### With Nav2
```bash
# Subscribe to joint states for odometry
ros2 topic echo /device1/get_joint_state
```

---

**🎯 System ready for 150Hz real-time motor control!**
