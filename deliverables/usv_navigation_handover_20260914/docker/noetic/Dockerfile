###############################################################################
# ROS Noetic development image for ros_motion_planning
###############################################################################
ARG BASE_IMAGE=osrf/ros:noetic-desktop-full
ARG CUDA_BASE_IMAGE=nvidia/cuda:11.8.0-cudnn8-runtime-ubuntu20.04

# The CUDA 11.8 + cuDNN 8 runtime libraries.
FROM ${CUDA_BASE_IMAGE} AS cuda-runtime
# The ROS development image.
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG VERSION_NAME=noetic
ARG PROJECT_NAME=rmp

WORKDIR /${PROJECT_NAME}
SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# 1. apt dependencies not already provided by desktop-full.
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
        ros-${VERSION_NAME}-amcl \
        ros-${VERSION_NAME}-map-server \
        ros-${VERSION_NAME}-move-base \
        ros-${VERSION_NAME}-move-base-msgs \
        ros-${VERSION_NAME}-base-local-planner \
        ros-${VERSION_NAME}-navfn \
        ros-${VERSION_NAME}-costmap-2d \
        ros-${VERSION_NAME}-nav-core \
        ros-${VERSION_NAME}-ros-control \
        ros-${VERSION_NAME}-ros-controllers \
        ros-${VERSION_NAME}-twist-mux \
        ros-${VERSION_NAME}-robot-localization \
        ros-${VERSION_NAME}-velodyne-description \
        ros-${VERSION_NAME}-velodyne-gazebo-plugins \
        ros-${VERSION_NAME}-pointgrey-camera-description \
        ros-${VERSION_NAME}-flir-camera-description \
        ros-${VERSION_NAME}-cpr-onav-description \
        ros-${VERSION_NAME}-angles \
        ros-${VERSION_NAME}-cmake-modules \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. Conan 1.59 + conancenter remote.
# ---------------------------------------------------------------------------
ARG PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/
ENV PIP_INDEX_URL=${PIP_INDEX_URL} \
    PIP_DEFAULT_TIMEOUT=100
RUN pip3 install --no-cache-dir "conan==1.59.0" \
    && (conan remote add conancenter https://center.conan.io || true)



# ---------------------------------------------------------------------------
# 3. Build third-party dependencies.
# ---------------------------------------------------------------------------
# Cap build parallelism.
ARG BUILD_JOBS=8
ENV CONAN_CPU_COUNT=${BUILD_JOBS} \
    MAKEFLAGS=-j${BUILD_JOBS} \
    CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}
COPY 3rd/conanfile.py 3rd/conaninfo.txt /${PROJECT_NAME}/3rd/
RUN cd /${PROJECT_NAME}/3rd \
    && conan profile new default --detect --force \
    && conan profile update settings.compiler.libcxx=libstdc++11 default \
    && conan install . --output-folder=build --build=missing \
        -s compiler.libcxx=libstdc++11 \
    && find /root/.conan/data \( -name "*.so" -o -name "*.so.*" \) \
        -exec ln -sf {} /usr/local/lib/ \; \
    && ldconfig

# CUDA 11.8 + cuDNN 8 runtime libraries.
COPY --from=cuda-runtime /usr/local/cuda-11.8/targets/x86_64-linux/lib /usr/local/cuda-11.8/lib64
COPY --from=cuda-runtime /usr/lib/x86_64-linux-gnu/libcudnn*.so* /usr/lib/x86_64-linux-gnu/
ENV LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:/usr/lib/x86_64-linux-gnu

# Onnxruntime library.
ARG GITHUB_MIRROR=https://gh-proxy.com/https://github.com/
RUN pip3 install --no-cache-dir "cmake==3.28.4" \
    && curl -fsSL "${GITHUB_MIRROR}microsoft/onnxruntime/releases/download/v1.12.0/onnxruntime-linux-x64-gpu-1.12.0.tgz" \
        -o /tmp/ort.tgz \
    && tar -xzf /tmp/ort.tgz -C /tmp \
    && mkdir -p /usr/local/include/onnxruntime \
    && cp -r /tmp/onnxruntime-linux-x64-gpu-1.12.0/include/. /usr/local/include/onnxruntime/ \
    && cp -a /tmp/onnxruntime-linux-x64-gpu-1.12.0/lib/. /usr/local/lib/ \
    && ldconfig \
    && rm -rf /tmp/ort.tgz /tmp/onnxruntime-linux-x64-gpu-1.12.0

# Python dependency.
RUN pip3 install --no-cache-dir "matplotlib==3.5.1"

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
# 4. Environment: source ROS for every shell flavour.
# ---------------------------------------------------------------------------
RUN printf '%s\n' \
        "source /opt/ros/${VERSION_NAME}/setup.bash" \
        'export ROS_MASTER_URI=http://localhost:11311' \
        'export ROS_HOSTNAME=localhost' \
        'export DISABLE_ROS1_EOL_WARNINGS=1' \
        "[ -f /${PROJECT_NAME}/devel/setup.bash ] && source /${PROJECT_NAME}/devel/setup.bash" \
        > /etc/profile.d/${PROJECT_NAME}.sh \
    && chmod +x /etc/profile.d/${PROJECT_NAME}.sh \
    && grep -q "profile.d/${PROJECT_NAME}.sh" /root/.bashrc 2>/dev/null \
        || echo "source /etc/profile.d/${PROJECT_NAME}.sh" >> /root/.bashrc

ENV BASH_ENV=/etc/profile.d/${PROJECT_NAME}.sh \
    ROS_MASTER_URI=http://localhost:11311 \
    ROS_HOSTNAME=localhost \
    DISABLE_ROS1_EOL_WARNINGS=1

CMD ["/bin/bash"]
