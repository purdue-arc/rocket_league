FROM osrf/ros:melodic-desktop-full

# install tools and dependencies from apt
RUN apt update && apt upgrade -y && \
    apt install -y sudo \
    nano vim zsh git curl wget \
    net-tools putils-ping \
    x11-apps mesa-utils \
    python-dev python3-dev \
    python-pip python3-pip \
    python-catkin-tools pylint \
    python-pygame python-box2d \
    intltool gstreamer1.0-gtk3 \
    libgtk-3-dev libnotify-dev libgstreamer1.0 \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    libcanberra-gtk3-module && \
    apt autoremove -y

# TODO check if following are needed:
# libcudnn8
# libnvinfer7
# tensorflow-gpu (pip)
# python3-libnvinfer (this and follwing unlikely)
# libcudnn8-dev
# libnvinfer-dev
# pycuda

# TODO check opengl forwarding

# install tools from pip3
RUN pip3 install --upgrade pip && \
    pip3 install rospkg \
    scipy numpy \
    matplotlib pandas \
    tensorflow torch \
    jupyterlab \
    gym[box2d]

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
RUN curl -fsSLo aravis-0.6.4.tar.xz http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.4.tar.xz && \
    echo "b595a4724da51d0fdb71f2b6e2f1e12f328e423155c3e84607ee2ce704f516bd  aravis-0.6.4.tar.xz" | sha256sum --check && \
    tar xf aravis-0.6.4.tar.xz && cd aravis-0.6.4 && \
    ./configure && make && make install && ldconfig
USER $USER
WORKDIR /home/$USER

# install ohmyzsh
RUN sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
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

