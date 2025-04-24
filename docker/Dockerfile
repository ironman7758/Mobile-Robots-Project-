FROM osrf/ros:jazzy-desktop-full
##Install any updates
RUN apt update && apt upgrade -y
##Install Packages for basic functionality
RUN apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy python3-colcon-common-extensions 

#Install AriaCoda
RUN git clone https://github.com/reedhedges/AriaCoda.git /workspace/AriaCoda
WORKDIR /workspace/AriaCoda
RUN sudo make && sudo make install-default 
RUN export LD_LIBRARY_PATH=`pwd`/lib
ADD ariaNode /workspace/ariaNode
WORKDIR /workspace/ariaNode
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install"
##################

#Install Sick lidar packages
RUN apt update && apt install ros-jazzy-sick-scan-xd -y
#Install Navsat driver
RUN apt update && apt install ros-jazzy-nmea-navsat-driver -y 
#Install common gps modules
RUN apt update && apt install ros-jazzy-gps-msgs ros-jazzy-gps-tools -y


##Final setup
WORKDIR /workspace
ENV SHELL=/bin/[bash]
ENV ROS_DOMAIN_ID=2
# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************


#########Package config files#######
#This is a specific config for the ps4 controller so joy can read it
ADD configs/ps4.config.yaml /opt/ros/jazzy/share/teleop_twist_joy/config/