##########################################################################
FROM tb5zhh/icra-2023-client-base-carto:v2.0.0

WORKDIR /opt/ep_ws
ENV ENV_ROBOT_MODE=sim

# copy sources
COPY src ./src

# install dependencies
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/realsense.list
RUN apt-get update \
    &&  source /opt/workspace/devel_isolated/setup.bash \
    &&  ROS_OS_OVERRIDE=ubuntu:20.04:focal DEBIAN_FRONTEND=noninteractive \       
        rosdep install --from-paths src --ignore-src --rosdistro noetic -y \
    &&  apt-get install -y --no-install-recommends \
            python3-tf-conversions \
            ros-noetic-depthimage-to-laserscan \
            ros-noetic-global-planner \
            ros-noetic-map-server \
    &&  pip3 install -i https://mirrors.bfsu.edu.cn/pypi/web/simple \
            scipy \
    &&  rm -rf /var/lib/apt/lists/* \
    &&  apt-get clean

# copy scripts
COPY build.sh start.sh ./

# build
RUN ./build.sh

CMD ./start.sh
