ARG CUDA_VERSION=11.7.1
ARG CUDNN_VERSION=8
ARG UBUNTU_VERSION=22.04

FROM nvidia/cuda:${CUDA_VERSION}-cudnn${CUDNN_VERSION}-devel-ubuntu${UBUNTU_VERSION}

# disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install cudnn8 and move necessary header files to cuda include directory
RUN apt-get update && \
	cp /usr/include/cudnn_version.h /usr/local/cuda/include && \
	cp /usr/include/cudnn.h /usr/local/cuda/include/ && \
	rm -rf /var/lib/apt/lists/*


# Fundamentals
RUN apt-get update && apt-get install -y \
        bash-completion \
        build-essential \
        ca-certificates \
        clang-format \
        cmake \
        curl \
        git \
        gnupg2 \
        locales \
        lsb-release \
        rsync \
        software-properties-common \
        wget \
        vim \
        unzip \
        mlocate \
	libgoogle-glog-dev \
        && rm -rf /var/lib/apt/lists/*

# Install libtorch
# RUN wget https://download.pytorch.org/libtorch/cu113/libtorch-cxx11-abi-shared-with-deps-1.11.0%2Bcu113.zip && \
#         unzip libtorch-cxx11-abi-shared-with-deps-1.11.0+cu113.zip && \
#         rm -rf libtorch-cxx11-abi-shared-with-deps-1.11.0+cu113.zip 

# Python basics
ENV PYTHON_VERSION=3.10
RUN apt-get update && apt-get install -y \
        python${PYTHON_VERSION} \
        python3-flake8 \
        python3-opencv \
        python3-pip \
        python3-pytest-cov \
        python3-setuptools \
        && rm -rf /var/lib/apt/lists/*

# Python3 (PIP)
RUN python3 -m pip install -U \
        argcomplete \
        autopep8 \
        flake8 \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        onnx \
        pytest-repeat \
        pytest-rerunfailures \
        pytest \
        pydocstyle
        
# Setup ROS2 humble
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8

RUN locale-gen en_US en_US.UTF-8 \
        && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
        && apt-get update && apt-get install -y software-properties-common \
        && add-apt-repository universe \
        && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
        && sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'      

RUN apt-get update && apt-get install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-${ROS_DISTRO}-ros-base \
        ros-dev-tools \
        # ros-humble-camera-calibration-parsers \
        # ros-humble-camera-info-manager \
        # ros-humble-desktop \
        # ros-humble-launch-testing-ament-cmake \
        # ros-humble-rqt* \
        # ros-humble-v4l2-camera \
        ros-${ROS_DISTRO}-vision-msgs \
        # ros-humble-pcl-conversions \
        # ros-humble-sensor-msgs-py \
        # ros-humble-stereo-image-proc \
        # ros-humble-pcl-ros \
        # ros-humble-usb-cam \
        # && ros-${ROS_DISTRO}-ament-cmake-nose \
        ros-${ROS_DISTRO}-nav2-common \
        && rm -rf /var/lib/apt/lists/* \
        && rosdep init \
        && rosdep update
        
RUN python3 -m pip install -U \
        numpy==1.24.4 \
        numba \
        torch==2.0.0 \
        torchvision==0.15.1 \
        torchaudio==2.0.1 \
        spconv-cu117 \
        av2 \
        transforms3d==0.4.1 \
        nose \
        tensorflow \
        pyquaternion \
        kornia==0.5.8 

# Install OpenPCDet
COPY docker/build_OpenPCDet.sh /build_OpenPCDet.sh
RUN /bin/bash /build_OpenPCDet.sh

# Install pcdet_ros2
COPY pcdet_ros2 /ros2_ws/src/pcdet_ros2
RUN mkdir -p /ros2_ws/src \
        && git clone https://github.com/Box-Robotics/ros2_numpy -b humble /ros2_ws/src/ros2_numpy \
        && cd /ros2_ws \
        && apt-get update \
        && . /opt/ros/${ROS_DISTRO}/setup.sh \
        && rosdep install -i --from-path src --rosdistro humble -y \
        && colcon build --symlink-install --packages-select ros2_numpy pcdet_ros2 \
        && rm -rf /var/lib/apt/lists/*

COPY ./docker/entrypoint.sh /
COPY ./docker/run_pcdet.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
