#!/bin/bash
set -e

echo "======================================"
echo "Building Yahboomcar ROS2 Docker Image"
echo "======================================"
echo "This may take 15-20 minutes on Jetson Orin Nano"
echo ""

# Build the image
docker compose build

echo ""
echo "======================================"
echo "Build complete!"
echo "======================================"
echo "To run: ./run_docker.sh"
echo "Or manually: docker compose up"
