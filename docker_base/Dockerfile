FROM osrf/ros:jazzy-desktop

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
# WORKDIR /root/performance_ws/src/graduate_research/install/publisher_node/lib/publisher_node
# CMD ["/bin/bash", "-c", ". /root/performance_ws/src/graduate_research/install/setup.sh && ./publisher_node --node_name $NODE_NAME --topic_name $TOPIC_NAME -s $PAYLOAD_SIZE -p $PERIOD_MS"]