FROM ros:humble-ros-base

# create workspace
RUN mkdir -p /ros_ws
WORKDIR /ros_ws/

# 小鱼一键换源
RUN apt update \
    && apt install wget python3-yaml -y  \
    && echo "chooses:\n" > fish_install.yaml \
    && echo "- {choose: 5, desc: '一键配置:系统源(更换系统源,支持全版本Ubuntu系统)'}\n" >> fish_install.yaml \
    && echo "- {choose: 2, desc: 更换系统源并清理第三方源}\n" >> fish_install.yaml \
    && echo "- {choose: 1, desc: 添加ROS/ROS2源}\n" >> fish_install.yaml \
    && wget http://fishros.com/install  -O fishros && /bin/bash fishros \
    && rm -rf /var/lib/apt/lists/*  /tmp/* /var/tmp/*

# 初始化 rosdepc
RUN apt-get update && apt-get install python3-pip -y && \
    pip install rosdepc && \
    sudo rosdepc init  && \
    rosdepc update

# 安装 Clang
RUN apt-get -y install clang  && \
    apt-get -y install clangd  && \
    apt-get -y install clang-format && \
    apt-get clean && apt autoclean

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd ./Livox-SDK2/ && \
    mkdir build && \
    cd build && \
    cmake .. && make -j && \
    sudo make install

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.0/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh && \
    rm -rf /var/lib/apt/lists/*

# setup .zshrc
RUN echo 'export TERM=xterm-256color\n\
source /opt/ros/humble/setup.zsh\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"\n'\
>> /root/.zshrc

# source entrypoint setup
RUN sed --in-place --expression \
    '$isource "/opt/ros/humble/setup.sh"' \
    /ros_entrypoint.sh
