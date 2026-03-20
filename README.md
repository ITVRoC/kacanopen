[![Licence](https://img.shields.io/badge/licence-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![drivers_lgpl Licence](https://img.shields.io/badge/drivers__lgpl%20licence-LGPL%202.1%2B-blue.svg)](https://opensource.org/licenses/LGPL-2.1)
[![Build Status](https://api.travis-ci.org/KITmedical/kacanopen.svg?branch=master)](https://travis-ci.org/KITmedical/kacanopen)

# KaCanOpen - ROS 2 Jazzy

🚀 **This is a complete ROS 2 Jazzy conversion of the original KaCanOpen library**, providing high-performance CANopen communication for robotics applications.

KaCanOpen is an easy-to-use [CANopen](https://en.wikipedia.org/wiki/CANopen) stack, which consists of four parts:

* __Drivers:__ A wide range of hardware is supported using different CAN drivers. They have been developed by the [CanFestival project](http://www.canfestival.org/). Read [this](drivers_lgpl/README) for details.

* __Core:__ This is a library which implements basic CANopen protocols like [NMT](https://en.wikipedia.org/wiki/CANopen#Network_management_.28NMT.29_protocols), [SDO](https://en.wikipedia.org/wiki/CANopen#Service_Data_Object_.28SDO.29_protocol) and [PDO](https://en.wikipedia.org/wiki/CANopen#Process_Data_Object_.28PDO.29_protocol). As an example, you can easily fetch a value from a device (*uploading* in CANopen terminology) via `core.sdo.upload(node_id, index, subindex)`. It furthermore allows you to register callbacks on certain events or any incoming messages, so one can build arbitrary CANopen nodes (master or slave) using this library.

* __Master:__ This library is intended to be used for a master node. It detects all nodes in a network and allows to access them via standardized [CiA® profiles](http://www.can-cia.org/can-knowledge/canopen/canopen-profiles/). For example, on a motor device (profile CiA® 402) you can simply call `mymotor.set_entry("Target velocity", 500)`. A main feature of this library is transparent SDO/PDO access to dictionary entries: By default a value is fetched and set via SDO, but you can configure PDO mappings to instead keep the value up-to-update in background via (more lightweight) PDO messages. The call itself (`mymotor.set_entry("Target velocity", 500)`) keeps unchanged.

* __ROS 2 Bridge:__ **Fully converted to ROS 2 Jazzy** with complete `rclcpp` integration, hardware interface support, and high-performance optimizations. This library provides a bridge to a [ROS 2](https://docs.ros.org/en/jazzy/) network, making KaCanOpen especially interesting for robotics. After setting up the CANopen network, the library can publish slave nodes so they are accessible through ROS 2 messages and services.

## 🎯 Key Features (ROS 2 Version)

- ✅ **Complete ROS 2 Jazzy compatibility** with `rclcpp`
- ✅ **High-performance 150Hz topic publishing** with shared memory optimization
- ✅ **Full hardware interface support** (`ros2_control` compatible)
- ✅ **Docker containerized deployment** for easy robot integration
- ✅ **Launch file integration** with parameter management
- ✅ **Real-time CAN communication** with multiple motor support
- ✅ **Automatic device detection** and configuration

## 🚀 Quick Start (ROS 2 + Docker)

### Prerequisites

- Docker and Docker Compose
- CAN hardware interface (e.g., `can0`)
- ROS 2 Jazzy (for development, not required for Docker deployment)

### 1. Clone and Configure

```bash
git clone https://github.com/your-repo/kacanopen.git
cd kacanopen

# Configure environment variables
cp .env.example .env
# Edit .env to match your setup:
# BUSNAME=can0
# BAUDRATE=1M
# ROS_DOMAIN_ID=0
```

### 2. Deploy with Docker

```bash
# Build and run the complete system
docker compose up

# Or run in background
docker compose up -d

# View logs
docker compose logs -f
```

### 3. Verify Operation

```bash
# In another terminal, check ROS 2 topics
docker compose exec kacanopen bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"

# Monitor joint states at 150Hz
docker compose exec kacanopen bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic hz /device1/get_joint_state"

# Check motor status
docker compose exec kacanopen bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /device1/get_joint_state"
```

## 📡 ROS 2 Topics and Services

### Published Topics

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/device{N}/get_joint_state` | `sensor_msgs/msg/JointState` | 150Hz | Motor position, velocity, effort |
| `/device{N}/profile_velocity_mode` | `std_msgs/msg/Int32` | 150Hz | Current velocity mode status |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/device{N}/set_joint_state` | `sensor_msgs/msg/JointState` | Set target joint positions/velocities |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/reset_motors` | `std_srvs/srv/Trigger` | Full CAN reset and motor re-initialization |

*Note: `{N}` represents the CANopen node ID (e.g., device1, device3, device4, device6)*

> For command examples (sending velocity, monitoring topics, resetting motors), see the [Quick Start Guide](QUICK_START.md).

## ⚙️ Configuration

### Environment Variables (.env file)

```bash
# CAN Interface Configuration
BUSNAME=can0                    # CAN interface name
BAUDRATE=1M                     # CAN baudrate (125K, 250K, 500K, 1M)

# Motor Control Parameters
ACCELERATION=10000              # Motor acceleration
DECELERATION=20000              # Motor deceleration
RESET_MOTORS_FLAG=false         # Reset motors on startup

# ROS 2 Configuration
ROS_DOMAIN_ID=0                 # ROS 2 domain ID
```

### Launch Parameters

The system uses ROS 2 launch files with parameter support:

```bash
# Manual launch with custom parameters
ros2 launch kacanopen espeleo_kacanopen_launch.py \
    busname:=can0 \
    baudrate:=1M \
    acceleration:=10000 \
    deceleration:=20000 \
    reset_motors_flag:=false
```

## 🏗️ Building from Source (ROS 2)

### Prerequisites

- ROS 2 Jazzy
- C++17 compiler (GCC >= 7.0)
- CMake >= 3.8
- Boost >= 1.46.1

### Build Instructions

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/your-repo/kacanopen.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build with colcon
colcon build --packages-select kacanopen \
    --cmake-args \
    -DDRIVER=socket \
    -DBUILD_ALL_DRIVERS=On \
    -DINSTALL_EXAMPLES=On \
    -DNO_ROS=Off

# Source the workspace
source install/setup.bash
```

### Supported Drivers

- `socket` - SocketCAN (recommended for Linux)
- `virtual` - Virtual CAN for testing
- `serial` - Serial CAN adapters
- `lincan` - LinCAN driver
- `peak_linux` - PEAK-System CAN adapters

## 🔧 Hardware Interface (ros2_control)

The system provides a complete ROS 2 hardware interface compatible with `ros2_control`:

### Hardware Interface Components

- **KaCanopenHardware**: Main hardware interface class
- **KaCanopenManager**: Motor management and coordination
- **KaCanopenMotor**: Individual motor control and state

### Usage with ros2_control

```yaml
# robot_description.yaml
hardware:
  - name: kacanopen_system
    type: kacanopen/KaCanopenHardware
    parameters:
      busname: can0
      baudrate: 1M
      motor_names: ["motor1", "motor2", "motor3", "motor4"]
```

## 🐳 Docker Configuration

### High-Performance Optimizations

The Docker setup includes several optimizations for real-time performance:

```yaml
# docker-compose.yml highlights
services:
  kacanopen:
    privileged: true
    network_mode: host          # Direct network access
    ipc: host                   # Shared memory for high-frequency communication
    pid: host                   # Shared PID namespace for better performance
    volumes:
      - /dev/shm:/dev/shm      # Shared memory for high-performance IPC
    environment:
      - ROS_LOCALHOST_ONLY=1    # Optimize for local communication
      - RCUTILS_LOGGING_BUFFERED_STREAM=1  # Reduce logging overhead
```

### Performance Tuning

The system is optimized for 150Hz real-time operation:

- **Shared memory IPC** for minimal latency
- **Host networking** for direct CAN access
- **Buffered logging** to reduce overhead
- **Real-time scheduling** with `ulimit -r 99`

## 🔍 Troubleshooting

### Common Issues

1. **CAN Interface Not Found**
   ```bash
   # Check CAN interface
   ip link show can0
   
   # Bring up CAN interface
   sudo ip link set can0 up type can bitrate 1000000
   ```

2. **Permission Denied**
   ```bash
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   
   # Or run with sudo
   sudo docker compose up
   ```

3. **Low Topic Frequency**
   - Ensure `ipc: host` and `network_mode: host` in docker-compose.yml
   - Check `/dev/shm` volume mount
   - Verify `ROS_LOCALHOST_ONLY=1` environment variable

4. **Motor Not Responding**
   - Check CAN bus termination
   - Verify baudrate matches motor configuration
   - Check motor node ID configuration

### Debug Commands

```bash
# Check CAN traffic
candump can0

# Monitor ROS 2 topics
ros2 topic list
ros2 topic hz /device1/get_joint_state
ros2 topic echo /device1/get_joint_state

# Check motor services
ros2 service list
ros2 service call /reset_motors std_srvs/srv/Trigger "{}"
```

## 📊 Performance Metrics

- **Topic Frequency**: 150Hz (optimized from original 5Hz)
- **Latency**: < 1ms with shared memory IPC
- **CPU Usage**: ~5-10% on modern hardware
- **Memory Usage**: ~50MB container footprint

## 🔄 Migration from ROS 1

This version is a complete conversion from ROS 1. Key changes:

- **ROS 1 → ROS 2**: Complete API migration to `rclcpp`
- **Catkin → Ament**: Build system updated to `ament_cmake`
- **Hardware Interface**: Updated to ROS 2 `hardware_interface`
- **Launch Files**: Converted to Python-based ROS 2 launch
- **Parameters**: Updated to ROS 2 parameter system
- **Docker**: Added containerized deployment

## 📚 Documentation

- [Installation Guide](doc/Installation.md)
- [API Documentation](https://kitmedical.github.io/kacanopen/)
- [Hardware Interface Guide](doc/HardwareInterface.md)
- [Docker Deployment Guide](doc/Docker.md)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test with real hardware
5. Submit a pull request

## 📄 License

- **Core, Master and ROS Bridge**: [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
- **Drivers**: [LGPLv2.1+](https://opensource.org/licenses/LGPL-2.1) (from CanFestival)

## 🙏 Acknowledgments

- Original KaCanOpen developers at KIT Medical
- CanFestival project for CAN drivers
- ROS 2 community for the excellent framework
- Contributors to this ROS 2 conversion

---

**🎯 Ready for production use with ROS 2 Jazzy and real-time robotics applications!**