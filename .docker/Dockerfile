# Install core dependencies need for ros2 humble
ARG ROS_DISTRO=humble
FROM osrf/ros:humble-desktop-full 

LABEL maintainer="Marcus Rosette"
LABEL maintainer-email="rosettem@oregonstate.edu"

# Set environment
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

# Make directories
RUN mkdir --p ~/ros2_ws/src
WORKDIR /root/ros2_ws

COPY . /

# Install Dependencies
RUN apt update \
    apt install -y \ 
    python3-pip \
    python3-venv \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget \
    git

WORKDIR /root/ros2_ws
COPY . /root/ros2_ws/

# Update rosdeps
RUN cd /root/ros2_ws/src \
    rosdep install -y \
    --from-paths src \ 
    --ignore-src \ 
    --skip-keys " \
    fastcdr \ 
    rti-connext-dds-6.0.1 \
    urdfdom_headers"

RUN apt upgrade -y

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "echo 'Terminal sourced using /opt/ros/humble/setup.bash'" >> ~/.bashrc

WORKDIR /root/ros2_ws/src