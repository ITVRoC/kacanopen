# Use ROS Noetic as base image
FROM osrf/ros:noetic-desktop-full-focal

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    can-utils \
    net-tools \
    iputils-ping \
    iproute2 \
    kmod \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Copy the kacanopen source code
COPY . /catkin_ws/src/kacanopen/

# Install ROS dependencies
RUN cd /catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y

# Set the default CAN driver to socket (virtual CAN interface)
ENV DRIVER=socket
ENV BUSNAME=vcan0
ENV BAUDRATE=500000

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    catkin_make -DDRIVER=socket"

# Source setup files in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Create a script to setup virtual CAN interface
RUN echo '#!/bin/bash\n\
# Setup virtual CAN interface\n\
modprobe vcan\n\
ip link add dev vcan0 type vcan\n\
ip link set up vcan0\n\
echo "Virtual CAN interface vcan0 is ready"\n\
ip link show vcan0' > /setup_vcan.sh && \
    chmod +x /setup_vcan.sh

# Set working directory
WORKDIR /catkin_ws

# Default command
CMD ["/bin/bash"]
