FROM nvidia/cuda:11.3.0-devel-ubuntu20.04
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
        sudo tmux nano vim git wget net-tools iputils-ping x11-apps \
        mesa-utils python3-dev python3-pip python3-tk software-properties-common \
        pylint clang-format jq swig zsh \
        # ROS
        ros-${ROS_DISTRO}-desktop-full \
        python3-catkin-tools \
        # glvnd stuff
        libglvnd0 libgl1 libglx0 libegl1 libgles2 \
        # pytorch deps 
        libopenblas-base libopenmpi-dev && \
    if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y nvidia-jetpack; \
    fi && \
    # deep learning stuff
    pip3 install --upgrade pip && \
    pip3 install \
        scipy numpy matplotlib pandas \
        jupyterlab gym[box2d] && \
    # Clean Up to reduce image size
    rm -rf /var/lib/apt/lists/* /root/.cache/pip/

# install pyenv
RUN git clone git://github.com/yyuu/pyenv.git /home/${USER}/.pyenv
ENV PYENV_ROOT /home/${USER}/.pyenv
ENV PATH $PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH
RUN pyenv install 3.6.15
RUN pyenv global 3.6.15
RUN pyenv rehash

# install pytorch
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
        pip3 install torch==1.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html; \
    elif [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl && \
        pip3 install torch-1.10.0-cp36-cp36m-linux_aarch64.whl && \
        rm torch-1.10.0-cp36-cp36m-linux_aarch64.whl; \
    fi

# install project software dependencies
RUN pip3 install box2d stable-baselines3 && \
    apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
        python3-pygame \
        ros-${ROS_DISTRO}-apriltag-ros \
        ros-${ROS_DISTRO}-pointgrey-camera-driver && \
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
