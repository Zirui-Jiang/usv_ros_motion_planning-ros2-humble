###############################################################################
# ROS 2 Humble development image for ros_motion_planning (otter only)
###############################################################################
ARG BASE_IMAGE=ros:humble-perception

FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG VERSION_NAME=humble
ARG PROJECT_NAME=rmp

WORKDIR /${PROJECT_NAME}
SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# 1. apt dependencies: ROS 2 packages not bundled with the base image.
# ---------------------------------------------------------------------------
RUN apt-get update --fix-missing \
    && apt-get -y --no-install-recommends install \
        git \
        vim \
        psmisc \
        net-tools \
        iputils-ping \
        xterm \
        python3-pip \
        python-is-python3 \
        libxml2-dev \
        libxslt-dev \
        ninja-build \
        ruby \
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
# 2. Python dependency.
# ---------------------------------------------------------------------------
ARG PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/
ENV PIP_INDEX_URL=${PIP_INDEX_URL} \
    PIP_DEFAULT_TIMEOUT=100
RUN pip3 install --no-cache-dir "matplotlib"

# LLVM 19 toolchain.
ARG LLVM_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/llvm-apt
RUN mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://apt.llvm.org/llvm-snapshot.gpg.key \
        | gpg --batch --yes --dearmor -o /etc/apt/keyrings/llvm.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/llvm.gpg] ${LLVM_APT_MIRROR}/focal/ llvm-toolchain-focal-19 main" \
        > /etc/apt/sources.list.d/llvm-19.list \
    && apt-get -o Acquire::Retries=3 update \
    && apt-get -o Acquire::Retries=3 -y --no-install-recommends install \
        clang-19 \
        clangd-19 \
        clang-format-19 \
        clang-tidy-19 \
    && rm -rf /var/lib/apt/lists/* \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-19 100 \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-19 100 \
    && update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-19 100
    
# ---------------------------------------------------------------------------
# 3. Environment: source ROS 2 for every shell flavour.
# ---------------------------------------------------------------------------
RUN printf '%s\n' \
        "source /opt/ros/${VERSION_NAME}/setup.bash" \
        "[ -f /${PROJECT_NAME}/install/setup.bash ] && source /${PROJECT_NAME}/install/setup.bash" \
        > /etc/profile.d/${PROJECT_NAME}.sh \
    && chmod +x /etc/profile.d/${PROJECT_NAME}.sh \
    && grep -q "profile.d/${PROJECT_NAME}.sh" /root/.bashrc 2>/dev/null \
        || echo "source /etc/profile.d/${PROJECT_NAME}.sh" >> /root/.bashrc

ENV BASH_ENV=/etc/profile.d/${PROJECT_NAME}.sh

CMD ["/bin/bash"]
