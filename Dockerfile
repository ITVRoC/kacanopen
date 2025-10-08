# Use ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libboost-all-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    can-utils \
    iproute2 \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-transmission-interface \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-launch-ros \
    ros-${ROS_DISTRO}-launch \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /workspace
RUN mkdir -p /workspace/src

# Copy the kacanopen package
COPY . /workspace/src/kacanopen/

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Install dependencies
RUN cd /workspace && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace (core functionality + converted ROS 2 bridge)
RUN cd /workspace && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select kacanopen \
        --cmake-args \
        -DDRIVER=socket \
        -DBUILD_ALL_DRIVERS=On \
        -DINSTALL_EXAMPLES=On \
        -DNO_ROS=Off

# Setup environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /workspace/install/setup.bash" >> /root/.bashrc

# Set up CAN interface script
RUN echo '#!/bin/bash\n\
# Setup virtual CAN interface\n\
modprobe vcan || true\n\
ip link add dev vcan0 type vcan || true\n\
ip link set up vcan0 || true\n\
echo "Virtual CAN interface vcan0 is ready"\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Expose the workspace
VOLUME ["/workspace"]

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
