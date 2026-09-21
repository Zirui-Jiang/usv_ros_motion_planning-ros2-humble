###############################################################################
# Cloud Agent base image for the USV ROS 2 Humble navigation simulation.
#
# ROS 2 Humble targets Ubuntu 22.04 (jammy), so the default Cursor image
# (Ubuntu 24.04) cannot host it natively. This image is derived from the
# official ROS 2 Humble perception image and adds the ROS packages the
# workspace declares (Gazebo Classic 11, Nav2, rviz2, robot_localization, ...).
#
# System-level, slow, stable dependencies live here. Workspace dependency
# resolution (rosdep) and the colcon build run in the repository install step,
# where the checked-out source is available.
###############################################################################
ARG BASE_IMAGE=ros:humble-perception
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG VERSION_NAME=humble
SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# ROS 2 / Gazebo packages required by the workspace plus build & dev tooling.
# gazebo-ros-pkgs pulls in Gazebo Classic 11; rosdep/colcon come from ros-base.
# ---------------------------------------------------------------------------
RUN apt-get update --fix-missing \
    && apt-get -y --no-install-recommends install \
        git \
        vim \
        psmisc \
        net-tools \
        iputils-ping \
        python3-pip \
        python-is-python3 \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        build-essential \
        cmake \
        ninja-build \
        ruby \
        libxml2-dev \
        libxslt-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        ros-${VERSION_NAME}-navigation2 \
        ros-${VERSION_NAME}-nav2-bringup \
        ros-${VERSION_NAME}-nav2-costmap-2d \
        ros-${VERSION_NAME}-robot-localization \
        ros-${VERSION_NAME}-robot-state-publisher \
        ros-${VERSION_NAME}-joint-state-publisher \
        ros-${VERSION_NAME}-xacro \
        ros-${VERSION_NAME}-rviz2 \
        ros-${VERSION_NAME}-gazebo-ros-pkgs \
        ros-${VERSION_NAME}-gazebo-ros2-control \
        ros-${VERSION_NAME}-gazebo-plugins \
        ros-${VERSION_NAME}-pointcloud-to-laserscan \
        ros-${VERSION_NAME}-topic-tools \
        ros-${VERSION_NAME}-angles \
        ros-${VERSION_NAME}-qt-dotgraph \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Python packages used by the navigation algorithms and plotting utilities.
# ---------------------------------------------------------------------------
RUN pip3 install --no-cache-dir numpy pyyaml matplotlib

# ---------------------------------------------------------------------------
# Source ROS 2 (and the workspace overlay when present) for every shell.
# ---------------------------------------------------------------------------
RUN printf '%s\n' \
        "source /opt/ros/${VERSION_NAME}/setup.bash" \
        '[ -f "${CURSOR_WORKSPACE:-/workspace}/install/setup.bash" ] && source "${CURSOR_WORKSPACE:-/workspace}/install/setup.bash"' \
        > /etc/profile.d/usv_ros.sh \
    && chmod +x /etc/profile.d/usv_ros.sh \
    && grep -q "profile.d/usv_ros.sh" /root/.bashrc 2>/dev/null \
        || echo "source /etc/profile.d/usv_ros.sh" >> /root/.bashrc

ENV BASH_ENV=/etc/profile.d/usv_ros.sh

CMD ["/bin/bash"]
