FROM osrf/ros:melodic-desktop-full

# install tools and dependencies from apt
RUN apt update && apt upgrade -y && apt autoremove -y
RUN apt install -y sudo \
    nano vim zsh git python-pip \
    x11-apps iputils-ping \
    python-catkin-tools pylint \
    python-pygame python-box2d \
    ros-melodic-apriltag-ros \
    intltool gstreamer1.0-gtk3 \
    libgtk-3-dev libnotify-dev libgstreamer1.0 \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    libcanberra-gtk3-module

# create user
ARG USER=arc
ARG PW=password
ARG UID=1000
ARG GID=1000
RUN groupadd -o -g $GID $USER && \
    useradd -om -u $UID -g $GID -G sudo -s /bin/bash $USER && \
    echo "$USER:$PW" | chpasswd
USER $USER
WORKDIR /home/$USER

# install aravis
USER root
WORKDIR /aravis
RUN curl -fsSLo aravis-0.6.4.tar.xz http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.4.tar.xz && \
    echo "b595a4724da51d0fdb71f2b6e2f1e12f328e423155c3e84607ee2ce704f516bd  aravis-0.6.4.tar.xz" | sha256sum --check && \
    tar xf aravis-0.6.4.tar.xz && cd aravis-0.6.4 && \
    ./configure && make && make install && ldconfig
USER $USER
WORKDIR /home/$USER

# install zsh
RUN sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-completions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-completions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k/gitstatus/install 
ARG FILES=files
COPY --chown=$USER $FILES /home/$USER/

# rosdep update
RUN echo "Fetching rosdep definitions..." && rosdep update
