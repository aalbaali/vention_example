###########################################
# This is an auto generated Dockerfile for ros:robot
# generated from docker_images/create_ros_image.Dockerfile.em
#
# Obtained from https://github.com/osrf/docker_images/blob/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/melodic/ubuntu/bionic/robot/Dockerfile
###########################################
FROM ros:melodic-ros-base-bionic AS base

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-robot=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

CMD ["bash"]

###########################################
# Minimal packages installed
###########################################
FROM base AS minimal

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/EST /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

# Set timezone
ENV TZ="Canada/Eastern"
RUN date

# Install ROS
RUN apt-get update && apt-get install -y \
    curl \
    dirmngr \
    gnupg2 \
    lsb-release \
    sudo \
  && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
  && apt-get update && apt-get install -y \
    ros-melodic-ros-base \
  && rm -rf /var/lib/apt/lists/*

# Setup environment
ENV LD_LIBRARY_PATH=/opt/ros/melodic/lib
ENV ROS_DISTRO=melodic
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=1
ENV PATH=/opt/ros/melodic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PYTHONPATH=/opt/ros/melodic/lib/python3/dist-packages

CMD ["bash"]

###########################################
#  Develop image 
###########################################
FROM base AS dev
SHELL ["/bin/bash", "-c", "-l"]

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG WORKSPACE=ros_ws

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Set user to non-root user
USER $USERNAME

RUN apt-get update && apt-get install -y \
    vim \
    neovim \
    dialog \
    apt-utils \
    sudo \
    build-essential \
    gcc \
    g++ \
    clang \
    make \
    bash-completion \
    cmake \
    gdb \
    git \
    pylint \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-sensor-msgs \
    wget \
    clangd \
    zsh \
    neovim \
    python3-pip \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool build-essential \
    # Install ros distro testing packages
    python3-autopep8 \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"

# Create workspace directory
RUN mkdir -p ~/$WORKSPACE/src

# Add ROS sourcing to zshrc
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Create catkin workspace
#RUN cd "/home/$USERNAME/$WORKSPACE" && catkin_init_workspace


ENV DEBIAN_FRONTEND=

CMD ["bash"]

