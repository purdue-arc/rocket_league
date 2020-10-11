FROM osrf/ros:melodic-desktop-full
RUN apt update && apt upgrade -y

RUN apt install -y sudo nano vim git \
    x11-apps iputils-ping \
    python-catkin-tools python-pygame \
    zsh curl ninja-build python3-pip \
    libgtk-3-dev libnotify-dev libgstreamer1.0 \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    libcanberra-gtk3-module

ARG USER=arc
ARG PW=password
ARG UID=1000
ARG GID=1000

RUN groupadd -o -g $GID $USER && \
    useradd -om -u $UID -g $GID -G sudo -s /bin/bash $USER && \
    echo "$USER:$PW" | chpasswd && \
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python get-pip.py

# Install aravis
WORKDIR /aravis
RUN pip3 install meson && \
    curl -fsSLo aravis-0.8.1.tar.xz https://ftp.acc.umu.se/pub/GNOME/sources/aravis/0.8/aravis-0.8.1.tar.xz  && \
    echo "5fab57711f158f2eec19c32c2b1b87f4e3761df456f23bd330050d91e0f664aa  aravis-0.8.1.tar.xz" | sha256sum --check && \
    tar xf aravis-0.8.1.tar.xz && cd aravis-0.8.1 && \
    meson build -Dviewer=enabled && \
    cd build && ninja && ninja install && ldconfig

USER $USER
WORKDIR /home/$USER
RUN sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-completions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-completions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k/gitstatus/install && \
    rosdep update

COPY files /home/$USER/