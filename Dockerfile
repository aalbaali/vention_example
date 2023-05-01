##############################################
# Created from template ros.dockerfile.jinja
# And from althackst/dockerfiles
##############################################

###########################################
# Base image 
###########################################
FROM ubuntu:20.04 AS base

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
    ros-noetic-ros-base \
  && rm -rf /var/lib/apt/lists/*

# Setup environment
ENV LD_LIBRARY_PATH=/opt/ros/noetic/lib
ENV ROS_DISTRO=noetic
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=1
ENV PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
###########################################
#  Develop image 
###########################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
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
    python3-catkin-tools \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-rosbag \
    ros-sensor-msgs \
    wget \
    clangd \
    zsh \
    neovim \
    python3-pip \
    # Install ros distro testing packages
    python3-autopep8 \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized" \
  # Update pydocstyle
  && pip install --upgrade pydocstyle

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

# Create workspace directory
RUN mkdir -p ~/$WORKSPACE/

# Create a development directory
RUN mkdir -p ~/Dev

# Install latest stable eigen release
RUN git config --global http.sslverify false \
    && git clone https://gitlab.com/libeigen/eigen.git ~/Dev/external/eigen \
    && cd ~/Dev/external/eigen \
    && mkdir build && cd build \
    && cmake .. \
    && sudo make install \
    && git config --global http.sslverify false 

# Install fzf
RUN git clone https://github.com/junegunn/fzf.git ~/Dev/external/fzf \
      && cd ~/Dev/external/fzf \
      && ./install --all

# Clone custom workstation setup
RUN git clone https://github.com/aalbaali/workstation_setup.git ~/Dev/workstation_setup \
      && cd ~/Dev/workstation_setup \
      && sudo ./scripts/install_packages.sh

# Install custom workstation setup
RUN cd ~/Dev/workstation_setup \
      && rm ~/.bashrc \
      && rm ~/.zshrc \
      && if [ -f ~/.gitconfig ]; then rm ~/.gitconfig; fi \
      && ./scripts/post_install_setup.sh \
          --zsh \
          --bash \
          --functions \
          --git \
          --nvim \
          --nvim-setup \
          --clang_format \
          --gdb \
          --tmux \
          --tmux-setup

# Add ROS sourcing to zshrc
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.zsh ]; then source /opt/ros/${ROS_DISTRO}/setup.zsh; fi" >> /home/$USERNAME/.zshrc

ENV DEBIAN_FRONTEND=

CMD ["zsh"]

###########################################
#  OpenCV
###########################################
FROM dev AS opencv


ARG USERNAME
ARG USER_UID
ARG USER_GID

ENV DEBIAN_FRONTEND=noninteractive

RUN sudo apt-get update \
  && sudo apt-get install -y \
    libopencv-dev \
    python3-opencv \
  && sudo rm -rf /var/lib/apt/lists/* 

ENV DEBIAN_FRONTEND=

USER $USERNAME

CMD ["zsh"]

###########################################
#  Full image 
###########################################
FROM opencv AS full

ARG USERNAME
ARG USER_UID
ARG USER_GID

ENV DEBIAN_FRONTEND=noninteractive
# Install the full release
RUN sudo apt-get update \
  && sudo apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
  && sudo rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

USER $USERNAME

CMD ["zsh"]

###########################################
#  Full+Gazebo image 
###########################################
FROM full AS gazebo

ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG WORKSPACE

ENV DEBIAN_FRONTEND=noninteractive
# Install gazebo
RUN sudo apt-get update \
  && sudo apt-get install -y \
    ros-${ROS_DISTRO}-gazebo* \
  && sudo rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

USER $USERNAME 

CMD ["zsh"]

###########################################
#  Full+Gazebo+Nvidia image 
###########################################

FROM gazebo AS gazebo-nvidia

ARG USERNAME
ARG USER_UID
ARG USER_GID

################
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN sudo apt-get update \
 && sudo apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

USER $USERNAME 

CMD ["zsh"]
