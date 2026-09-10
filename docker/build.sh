#!/bin/bash

cd "$(dirname "$0")/.."

docker build \
    -t ros_motion_planning:noetic \
    -f docker/Dockerfile .
