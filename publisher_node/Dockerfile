FROM osrf/ros:jazzy-desktop

# これとは別に、docker-compose.ymlから環境変数を受け取る.
ENV ROS_DISTRO=jazzy
ENV CMAKE_PREFIX_PATH=/opt/ros/jazzy

# ワークスペースを作成し、本プロジェクトをクローン
RUN apt update \
    && mkdir -p ~/performance_ws/src \
    && cd ~/performance_ws/src \
    && git clone https://github.com/tropical-summer/graduate_research.git

# プロジェクトのビルド
# RUNコマンドではbashファイルを叩けないしsourceもつかえない。→　.でshファイルを叩く
RUN git clone https://github.com/jarro2783/cxxopts.git \
    && . /opt/ros/jazzy/setup.sh \
    && cd cxxopts \
    && mkdir build && cd build \
    && cmake .. \
    && make && sudo make install \
    && cd ../.. && cd root/performance_ws/src/graduate_research \
    && colcon build

# コンテナを起動するときのコマンド　ENVを扱うために、exec形式でありながらシェル形式を用いる(/bin/bash -c)
WORKDIR /root/performance_ws/src/graduate_research/install/publisher_node/lib/publisher_node
CMD ["/bin/bash", "-c", ". /root/performance_ws/src/graduate_research/install/setup.sh && ./publisher_node --node_name $NODE_NAME --topic_name $TOPIC_NAME -s $PAYLOAD_SIZE -p $PERIOD_MS"]


    







# RUN apt update && apt install locales \
#     && locale-gen ja_JP ja_JP.UTF-8 \
#     && update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8

# ENV LANG=ja_JP.UTF-8

# RUN apt install software-properties-common -y && add-apt-repository universe 

# RUN apt update && apt install curl -y \
#     #### && curl -sSL https://raw.githubsercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
#     && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null \
#     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
#     #### && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
#     && apt update

# RUN apt install ros-$ROS_DISTRO-desktop ros-dev-tools python3-argcomplete -y \
#     && rosdep init && rosdep update

