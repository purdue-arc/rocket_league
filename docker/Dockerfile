FROM nvidia/cuda:11.4.2-devel-ubuntu20.04
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
        # anaconda deps 
        bzip2 ca-certificates libglib2.0-0 libxext6 libsm6 libxrender1 \
        mercurial subversion libopenblas-dev && \
    # deep learning stuff
    pip3 install --upgrade pip && \
    pip3 install \
        scipy numpy matplotlib pandas \
        jupyterlab gym[box2d] && \
    # Clean Up to reduce image size
    rm -rf /var/lib/apt/lists/* /root/.cache/pip/


# install anaconda
#WORKDIR /anaconda
#RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
#        wget https://repo.anaconda.com/archive/Anaconda3-2021.05-Linux-x86_64.sh -O anaconda.sh; \
#    elif [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
#        wget https://repo.anaconda.com/archive/Anaconda3-2021.05-Linux-aarch64.sh -O anaconda.sh; \
#    fi && \
#    chmod +x anaconda.sh && \
#    ./anaconda.sh -b -p /opt/conda && \
#    rm anaconda.sh && \
#    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
#    find /opt/conda/ -follow -type f -name '*.a' -delete && \
#    find /opt/conda/ -follow -type f -name '*.js.map' -delete && \
#    /opt/conda/bin/conda clean -afy
#ENV PATH /opt/conda/bin:$PATH
#WORKDIR /root

# install pytorch
#ENV OLD_CMAKE_PREFIX_PATH $CMAKE_PREFIX_PATH
#ENV CMAKE_PREFIX_PATH /opt/conda
ENV USE_NCCL 0
ENV USE_DISTRIBUTED 0
ENV USE_QNNPACK 0
ENV USE_PYTORCH_QNNPACK 0
ENV TORCH_CUDA_ARCH_LIST 5.3;6.2;7.2
ENV PYTORCH_BUILD_VERSION 1.9
ENV PYTORCH_BUILD_NUMBER 1
#RUN conda install astunparse numpy ninja pyyaml mkl mkl-include setuptools \
#        cmake cffi typing_extensions future six requests dataclasses && \
#    git clone --recursive --branch release/1.9 https://github.com/pytorch/pytorch && \
#    cd pytorch && \
#    if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
#        wget https://gist.githubusercontent.com/dusty-nv/ce51796085178e1f38e3c6a1663a93a1/raw/bdb08d33760fc94ffbca20ed843c638228f8bb45/pytorch-1.9-jetpack-4.5.1.patch && \
#        git apply pytorch-1.9-jetpack-4.5.1.patch; \
#    fi && \
#    python3 setup.py install && \
#    cd .. && \
#    rf -rf pytorch
RUN git clone --recursive --branch release/1.9 https://github.com/pytorch/pytorch && \
    cd pytorch && \
    if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        wget https://gist.githubusercontent.com/dusty-nv/ce51796085178e1f38e3c6a1663a93a1/raw/bdb08d33760fc94ffbca20ed843c638228f8bb45/pytorch-1.9-jetpack-4.5.1.patch && \
        git apply pytorch-1.9-jetpack-4.5.1.patch; \
    fi && \
    pip3 install -r requirements.txt && \
    pip3 install scikit-build ninja && \
    python3 setup.py install && \
    cd .. && \
    rf -rf pytorch && /root/.cache/pip/
#ENV CMAKE_PREFIX_PATH $OLD_CMAKE_PREFIX_PATH

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
