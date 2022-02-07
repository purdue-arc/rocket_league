FROM nvidia/cuda:11.1.1-devel-ubuntu20.04
ARG TARGETPLATFORM
ENV ROS_DISTRO noetic

# Add ROS Keyserver
RUN apt clean && apt update && DEBIAN_FRONTEND=noninteractive apt install -y curl lsb-core && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    rm -rf /var/lib/apt/lists/*

# Update, Upgrade, Install, and Cleanup
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt upgrade -y && \
    DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y \
    # Basics
    sudo tmux nano vim git wget net-tools iputils-ping x11-apps htop \
    mesa-utils python3-dev python3-pip python3-tk software-properties-common \
    pylint clang-format jq swig zsh libeigen3-dev \
    # ROS
    ros-${ROS_DISTRO}-desktop-full \
    python3-catkin-tools \
    # glvnd stuff
    libglvnd0 libgl1 libglx0 libegl1 libgles2 && \
    # deep learning stuff
    pip3 install --upgrade pip && \
    pip3 install \
    scipy numpy matplotlib pandas \
    jupyterlab gym[box2d] autopep8 && \
    # Clean Up to reduce image size
    rm -rf /var/lib/apt/lists/* /root/.cache/pip/

# install pytorch [TODO On arm64]
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
    pip3 install torch==1.9.1+cu111 -f https://download.pytorch.org/whl/torch_stable.html && \
    rm -rf /root/.cache/pip/; \
    #elif [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
    #    wget https://nvidia.box.com/shared/static/h1z9sw4bb1ybi0rm3tu8qdj8hs05ljbm.whl -O torch-1.9.0-cp36-cp36m-linux_aarch64.whl && \
    #    apt update && \
    #    DEBIAN_FRONTEND=noninteractive apt install -y libopenblas-base libopenmpi-dev && \
    #    rm -rf /var/lib/apt/lists/* && \
    #    pip3 install Cython && \
    #    pip3 install torch-1.9.0-cp36-cp36m-linux_aarch64.whl; \
    fi

# install project software dependencies
RUN pip3 install pybullet stable-baselines3 && \
    apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
    python3-pygame \
    ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-pointgrey-camera-driver \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-rosserial-arduino \
    ros-${ROS_DISTRO}-rosbridge-server && \
    rm -rf /var/lib/apt/lists/* /root/.cache/pip/

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
