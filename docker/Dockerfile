FROM nvidia/cudagl:11.0-runtime-ubuntu18.04

# update, upgrade, and install basics
RUN apt update && apt upgrade -y && \
    DEBIAN_FRONTEND=noninteractive \
    apt install -y sudo \
    nano vim git curl wget \
    net-tools iputils-ping \
    x11-apps mesa-utils \
    python-dev python3-dev \
    python-pip python3-pip \
    pylint software-properties-common

# install Nvidia software for deep learning
RUN apt install -y --no-install-recommends \
    libcudnn8 libnvinfer7

# install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt update && \
    apt install -y \
    ros-melodic-desktop-full \
    python-catkin-tools

# install libraries for ROS and deep learning with python3
RUN pip3 install --upgrade pip && \
    pip3 install rospkg \
    scipy numpy \
    matplotlib pandas \
    tensorflow torch \
    jupyterlab \
    gym[box2d]

# install software dependencies
RUN apt install -y python-pygame python-box2d

# create user
ARG USER=arc
ARG PW=password
ARG UID=1000
ARG GID=1000
RUN groupadd -o -g $GID $USER && \
    useradd -om -u $UID -g $GID -G sudo -s /bin/bash $USER && \
    echo "$USER:$PW" | chpasswd

# install aravis
WORKDIR /aravis
RUN apt install -y intltool gstreamer1.0-gtk3 \
    libgtk-3-dev libnotify-dev libgstreamer1.0 \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    libcanberra-gtk3-module && \
    curl -fsSLo aravis-0.6.4.tar.xz http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.4.tar.xz && \
    echo "b595a4724da51d0fdb71f2b6e2f1e12f328e423155c3e84607ee2ce704f516bd  aravis-0.6.4.tar.xz" | sha256sum --check && \
    tar xf aravis-0.6.4.tar.xz && cd aravis-0.6.4 && \
    ./configure && make && make install && ldconfig
USER $USER
WORKDIR /home/$USER

# install zsh
RUN apt install zsh && \
    sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-completions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-completions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k/gitstatus/install
ARG FILES=files
COPY --chown=$USER $FILES /home/$USER/

# install ROS dependencies. To add a dependency, add it to the below variable, seperated by a space
ARG ROS_DEPS="apriltag_ros usb_cam"
RUN rosdep update
USER root
# Normally you can just do `rosdep install PACKAGE_NAME`, but it throws a fit if you tell it to install a package that's already installed
RUN rosdep resolve $ROS_DEPS | sed 's/#.*$//'| xargs apt install -y
USER $USER
