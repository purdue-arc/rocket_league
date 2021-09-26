ARG TARGETPLATFORM
FROM nvidia/cuda:11.4.1-devel-ubuntu20.04
ENV ROS_DISTRO noetic

# Add ROS Keyserver
RUN apt clean && apt update && DEBIAN_FRONTEND=noninteractive apt install -y curl lsb-core && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Update, Upgrade, Install, and Cleanup
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt upgrade -y && \
    DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y \
        # Basics
        sudo tmux nano vim git wget net-tools iputils-ping x11-apps \
        mesa-utils python3-dev python3-pip pylint software-properties-common \
        clang-format jq swig zsh \
        # ROS
        ros-${ROS_DISTRO}-desktop-full \
        python3-catkin-tools \
        # Aravis Dependencies
        intltool gstreamer1.0-gtk3 libgtk-3-dev libnotify-dev \
        libgstreamer1.0-0 libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
        libcanberra-gtk3-module libxml2-dev \
        # glvnd stuff
        libglvnd0 libgl1 libglx0 libegl1 libgles2 && \
    # deep learning stuff
    pip3 install \
        rospkg scipy numpy matplotlib pandas \
        # tensorflow \
        torch jupyterlab gym[box2d] && \
    # Clean Up to reduce image size
    rm -rf /var/lib/apt/lists/*

# install aravis
WORKDIR /aravis
RUN curl -fsSLo aravis-0.6.4.tar.xz http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.4.tar.xz && \
    echo "b595a4724da51d0fdb71f2b6e2f1e12f328e423155c3e84607ee2ce704f516bd  aravis-0.6.4.tar.xz" | sha256sum --check && \
    tar xf aravis-0.6.4.tar.xz && cd aravis-0.6.4 && \
    ./configure && make && make install && ldconfig && \
    cd / && rm -rf aravis

# install project software dependencies
RUN pip3 install box2d && \
    apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
        python3-pygame \
        ros-${ROS_DISTRO}-apriltag-ros

# create user
ARG USER=arc
ARG PW=password
ARG UID=1000
ARG GID=1000
RUN groupadd -o -g $GID $USER && \
    useradd -om -u $UID -g $GID -G sudo -s /bin/zsh $USER && \
    echo "$USER:$PW" | chpasswd
WORKDIR /home/$USER

# install zsh
USER $USER
RUN sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-completions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-completions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k/gitstatus/install
ARG FILES=files
COPY --chown=$USER $FILES /home/$USER/

CMD /bin/zsh
