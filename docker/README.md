# KaCanOpen Docker Setup

This directory contains Docker configuration files to easily build and run the KaCanOpen project in a containerized environment with ROS Noetic.

## Quick Start

1. **Build and start the environment:**
   ```bash
   cd docker
   docker-compose up --build -d
   ```

2. **Access the container:**
   ```bash
   docker-compose exec kacanopen bash
   ```

3. **Run the example program:**
   ```bash
   docker-compose exec kacanopen bash -c "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun kacanopen kacanopen_example_motor_and_io_bridge"
   ```

## Files Overview

- **`Dockerfile`**: Defines the Docker image with ROS Noetic, dependencies, and KaCanOpen
- **`docker-compose.yml`**: Orchestrates the containers with proper networking and volumes
- **`README.md`**: This documentation file

## Basic Docker Commands

| Command | Description |
|---------|-------------|
| `docker-compose up --build -d` | Build image and start containers in background |
| `docker-compose down` | Stop and remove containers |
| `docker-compose exec kacanopen bash` | Open a bash shell in the container |
| `docker-compose logs -f` | Show container logs |
| `docker-compose restart` | Restart containers |

## Examples

### Basic Usage
```bash
# Build and start containers
docker-compose up --build -d

# Open interactive shell
docker-compose exec kacanopen bash

# Stop containers
docker-compose down
```

### Running ROS Commands
```bash
# List ROS topics
docker-compose exec kacanopen bash -c "source /opt/ros/noetic/setup.bash && rostopic list"

# Show ROS nodes
docker-compose exec kacanopen bash -c "source /opt/ros/noetic/setup.bash && rosnode list"

# Run example
docker-compose exec kacanopen bash -c "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun kacanopen kacanopen_example_motor_and_io_bridge"
```

## Container Features

- **Base Image**: ROS Noetic Desktop Full
- **CAN Support**: Virtual CAN interface (vcan0) configured automatically
- **Dependencies**: All required packages (Boost, CMake, etc.) pre-installed
- **Networking**: Host networking for ROS communication
- **X11 Forwarding**: Support for GUI applications
- **Volume Mounts**: Source code mounted for live development

## CAN Interface Configuration

The container automatically sets up a virtual CAN interface (`vcan0`) which is suitable for testing and development. The default configuration uses:

- **Driver**: socket
- **Interface**: vcan0
- **Baudrate**: 500000

You can modify these settings in the `docker-compose.yml` file or override them when running the container.

## Troubleshooting

### Docker Permission Issues
If you encounter permission issues, make sure your user is in the docker group:
```bash
sudo usermod -aG docker $USER
# Log out and log back in
```

### X11 Display Issues
If GUI applications don't work, try:
```bash
xhost +local:docker
```

### CAN Interface Issues
If you need to use real CAN hardware, you may need to modify the docker-compose.yml to:
1. Add specific device mounts
2. Adjust the CAN driver configuration
3. Ensure proper privileges

### Build Issues
If the build fails:
1. Check that you have enough disk space
2. Ensure Docker daemon is running
3. Try rebuilding with `./launch.sh rebuild`

## Development Tips

1. **Live Editing**: Source code is mounted, so changes are reflected immediately
2. **Rebuilding**: Only rebuild when changing dependencies or Docker configuration
3. **Multiple Shells**: You can run `./launch.sh shell` multiple times for different terminals
4. **ROS Integration**: The container runs with host networking for seamless ROS communication

## Hardware Integration

To use real CAN hardware:

1. Connect your CAN adapter
2. Modify `docker-compose.yml` to mount the device (e.g., `/dev/ttyUSB0`)
3. Change the DRIVER environment variable to match your hardware
4. Restart the containers with `./launch.sh rebuild`

Example for Peak CAN adapter:
```yaml
environment:
  - DRIVER=peak_linux
devices:
  - /dev/pcanusb0:/dev/pcanusb0
```
