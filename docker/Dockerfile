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
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k

RUN echo 'export ZSH="/home/$USER/.oh-my-zsh"' > /home/$USER/.zshrc && \
    echo 'ZSH_THEME="powerlevel10k/powerlevel10k"' >> /home/$USER/.zshrc && \
    echo 'plugins=(git alias-finder colored-man-pages https://github.com/zsh-users/zsh-autosuggestions https://github.com/zsh-users/zsh-completions)' >> /home/$USER/.zshrc && \
    echo 'source $ZSH/oh-my-zsh.sh' >> /home/$USER/.zshrc && \
    echo 'source /ros_entrypoint.sh' >> /home/$USER/.zshrc && \
    echo 'set +e' >> /home/$USER/.zshrc && \
    echo 'source ~/.p10k.zsh' >> /home/$USER/.zshrc
COPY .p10k.zsh /home/$USER/.p10k.zsh
