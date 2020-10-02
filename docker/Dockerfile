FROM osrf/ros:melodic-desktop-full
RUN apt update && apt upgrade -y

RUN apt install -y sudo nano vim git \
    x11-apps iputils-ping \
    python-catkin-tools python-pygame \
    zsh curl

ARG USER=arc
ARG PW=password
ARG UID=1000
ARG GID=1000

RUN groupadd -o -g $GID $USER && \
    useradd -om -u $UID -g $GID -G sudo -s /bin/bash $USER && \
    echo "$USER:$PW" | chpasswd

USER $USER
WORKDIR /home/$USER
RUN sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-completions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-completions

COPY files /home/$USER/
