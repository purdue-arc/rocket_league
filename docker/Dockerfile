FROM osrf/ros:noetic-desktop-full

# Update, Upgrade, Install, and Cleanup
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt upgrade -y && \
    # basics
    DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y \
    sudo tmux nano vim git wget curl net-tools iputils-ping x11-apps htop \
    mesa-utils python3-dev python3-pip python3-tk software-properties-common \
    pylint clang-format jq swig zsh libeigen3-dev python3-catkin-tools python-is-python3 && \
    rm -rf /var/lib/apt/lists/*

# install project software dependencies
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
    ros-$ROS_DISTRO-apriltag-ros \
    ros-$ROS_DISTRO-pointgrey-camera-driver \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-camera-calibration \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-rosserial-arduino \
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-rosdoc-lite && \
    rm -rf /var/lib/apt/lists/*

# Install pip packages
WORKDIR /root
COPY requirements.in requirements.in
RUN pip install pip-tools && \
    pip-compile --output-file=requirements.txt --resolver=backtracking requirements.in && \
    pip install -r requirements.txt && \
    rm -rf /root/.cache/pip/ requirements.in requirements.txt

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
