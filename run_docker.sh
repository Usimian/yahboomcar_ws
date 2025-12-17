#!/bin/bash
set -e

echo "======================================"
echo "Starting Yahboomcar ROS2 Container"
echo "======================================"

# Allow X11 connections (for RViz if needed)
xhost +local:docker 2>/dev/null || true

# Start the container
docker compose up -d

echo ""
echo "======================================"
echo "Container started!"
echo "======================================"
echo "To view logs: docker compose logs -f"
echo "To open shell: ./shell_docker.sh"
echo "To stop: docker compose down"
