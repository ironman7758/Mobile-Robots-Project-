FROM osrf/ros:jazzy-desktop-full

RUN apt update && apt upgrade -y
RUN apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy python3-colcon-common-extensions 
RUN git clone https://github.com/reedhedges/AriaCoda.git /workspace/AriaCoda
WORKDIR /workspace/AriaCoda
RUN sudo make && sudo make install-default 
RUN export LD_LIBRARY_PATH=`pwd`/lib
ADD ariaNode /workspace/ariaNode
WORKDIR /workspace/ariaNode
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install"
WORKDIR /workspace
ENV SHELL=/bin/[bash]
ENV ROS_DOMAIN_ID=2
# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************