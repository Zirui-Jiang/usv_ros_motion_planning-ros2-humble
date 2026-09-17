#!/usr/bin/env bash

###############################################################################
# Centralized naming for the ros_motion_planning_pro docker dev environment.
# Sourced by run.sh / into.sh / stop.sh and consumed at image build time via
# --build-arg PROJECT_NAME. Change a value here and all scripts follow.
###############################################################################

# Project name: the bind-mount target inside the container (/rmp) and the
# prefix of the container name.
PROJECT_NAME="rmp"

# ROS distro / image variant: the image tag suffix and the container suffix.
VERSION_NAME="humble"

# Final image and container names, derived from the two above.
IMAGE="ros-motion-planning:${VERSION_NAME}"
CONTAINER="${PROJECT_NAME}_${VERSION_NAME}"

# Base image used to build the dev image (pulled from a mirror if needed).
BASE_IMAGE="ros:humble-perception"
