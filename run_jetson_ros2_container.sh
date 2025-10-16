#!/bin/bash
# Jetson ROS2 Container Runner (Official NVIDIA jetson-containers)
# Runs ROS2 container with native Argus camera support

CONTAINER_NAME="jetson_ros2_stereo"
IMAGE_NAME="dustynv/ros:humble-pytorch-l4t-r35.4.1"
WORKSPACE="/home/jay/projects/stereo_camera"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}Starting Jetson ROS2 Container...${NC}"
echo -e "${YELLOW}Image: ${IMAGE_NAME}${NC}"

# Check if container is already running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo -e "${GREEN}Container already running. Attaching...${NC}"
    docker exec -it $CONTAINER_NAME bash
    exit 0
fi

# Check if image exists, if not pull it
if ! docker images | grep -q "dustynv/ros"; then
    echo -e "${YELLOW}Pulling jetson-containers ROS2 image (this may take a while)...${NC}"
    docker pull $IMAGE_NAME
fi

# Run new container with full Jetson support
docker run -it --rm \
    --name $CONTAINER_NAME \
    --runtime nvidia \
    --network host \
    --privileged \
    --ipc=host \
    --pid=host \
    --device /dev/video0:/dev/video0 \
    --device /dev/video1:/dev/video1 \
    --device /dev/i2c-0:/dev/i2c-0 \
    --device /dev/i2c-1:/dev/i2c-1 \
    --device /dev/i2c-2:/dev/i2c-2 \
    --device /dev/i2c-7:/dev/i2c-7 \
    --device /dev/i2c-8:/dev/i2c-8 \
    --volume $WORKSPACE:/workspace \
    --volume /tmp/argus_socket:/tmp/argus_socket \
    --volume /run/systemd:/run/systemd:ro \
    --workdir /workspace \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=0 \
    $IMAGE_NAME \
    /bin/bash

echo -e "${GREEN}Container stopped.${NC}"
