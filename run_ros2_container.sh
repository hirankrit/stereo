#!/bin/bash
# ROS2 Humble Container Runner
# Runs ROS2 container with camera and GPU access

CONTAINER_NAME="ros2_stereo_camera"
IMAGE_NAME="ros2-jetson-camera:latest"
WORKSPACE="/home/jay/projects/stereo_camera"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Starting ROS2 Humble Container...${NC}"

# Check if container is already running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo -e "${GREEN}Container already running. Attaching...${NC}"
    docker exec -it $CONTAINER_NAME bash
    exit 0
fi

# Run new container
docker run -it --rm \
    --name $CONTAINER_NAME \
    --runtime nvidia \
    --gpus all \
    --network host \
    --privileged \
    --device /dev/video0:/dev/video0 \
    --device /dev/video1:/dev/video1 \
    --device /dev/i2c-0:/dev/i2c-0 \
    --device /dev/i2c-1:/dev/i2c-1 \
    --device /dev/i2c-2:/dev/i2c-2 \
    --device /dev/i2c-7:/dev/i2c-7 \
    --device /dev/i2c-8:/dev/i2c-8 \
    --volume $WORKSPACE:/workspace \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
    --volume /tmp/argus_socket:/tmp/argus_socket \
    --volume /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
    --volume /usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/tegra-egl:ro \
    --volume /etc/nv_tegra_release:/etc/nv_tegra_release:ro \
    --workdir /workspace \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=0 \
    --env LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra-egl:$LD_LIBRARY_PATH \
    --env GST_PLUGIN_SYSTEM_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0:/opt/nvidia/gstreamer-1.0 \
    $IMAGE_NAME \
    bash -c "source /opt/ros/humble/setup.bash && exec bash"

echo -e "${GREEN}Container stopped.${NC}"
