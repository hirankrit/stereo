# Dockerfile for ROS2 Humble + Jetson Camera Support
FROM arm64v8/ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Update and install essential packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # GStreamer and tools
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    # Python and OpenCV
    python3-opencv \
    python3-pip \
    python3-numpy \
    # ROS2 packages
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-image-pipeline \
    # Graphics libraries
    libegl1 \
    libgles2 \
    libgl1 \
    libglx0 \
    libglvnd0 \
    # Utilities
    v4l-utils \
    wget \
    curl \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
RUN mkdir -p /workspace

# Set working directory
WORKDIR /workspace

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set entrypoint - let the run script handle bash invocation
CMD ["/bin/bash"]
