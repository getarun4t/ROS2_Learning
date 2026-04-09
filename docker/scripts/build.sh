#!/bin/bash
set -e

# Get project root (2 levels up from scripts/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "Project root: $PROJECT_ROOT"

echo "Building base image..."
docker build -f "$PROJECT_ROOT/docker/dockerfiles/base.24.04.Dockerfile" \
    -t robotarm/base:24.04 "$PROJECT_ROOT"

echo "Building ROS2 image..."
docker build -f "$PROJECT_ROOT/docker/dockerfiles/ros2.Dockerfile" \
    -t robotarm/ros2:jazzy "$PROJECT_ROOT"