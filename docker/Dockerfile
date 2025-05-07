FROM osrf/ros:jazzy-desktop-full
##Install any updates
RUN apt update && apt upgrade -y
##Install Packages for basic functionality
RUN apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy python3-colcon-common-extensions xcb -y

#Install AriaCoda
RUN git clone https://github.com/reedhedges/AriaCoda.git /workspace/AriaCoda
WORKDIR /workspace/AriaCoda
RUN sudo make && sudo make install-default 
RUN export LD_LIBRARY_PATH=`pwd`/lib
ADD /ros_ws/src/ariaNode /ros_ws/src/ariaNode
WORKDIR /ros_ws/src/ariaNode
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install"
##################
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

#Install Sick lidar packages
RUN apt update && apt install ros-jazzy-sick-scan-xd -y
#Install Navsat driver
RUN apt update && apt install ros-jazzy-nmea-navsat-driver -y 

RUN apt update && apt install ros-jazzy-robot-localization -y

RUN apt update && apt install ros-jazzy-mapviz ros-jazzy-mapviz-plugins ros-jazzy-tile-map -y

RUN apt update && apt install ros-jazzy-navigation2 -y

RUN apt update && apt install ros-jazzy-nav2-bringup -y

RUN apt update && apt install ros-jazzy-nav2-minimal-tb*

RUN apt update && apt install ros-jazzy-ros-gz -y

#Install camera
RUN apt update && apt install ros-jazzy-depthai-ros -y

#Install common gps modules
RUN apt update && apt install ros-jazzy-gps-msgs ros-jazzy-gps-tools -y

RUN apt update && apt install ros-jazzy-phidgets-spatial -y
#simulation
RUN apt install ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-turtlebot3-bringup -y

##Final setup
WORKDIR /workspace
ENV SHELL=/bin/[bash]
ENV ROS_DOMAIN_ID=2 

RUN echo "export DISPLAY=:0.0" >> /etc/bash.bashrc
#VOLUME "$HOME/.Xauthority:/root/.Xauthority:rw"
# ********************************************************
# * Anything else you want to do like clean up goes here *  
# ********************************************************


#########Package config files#######
#This is a specific config for the ps4 controller so joy can read it
ADD configs/ps4.config.yaml /opt/ros/jazzy/share/teleop_twist_joy/config/
#########Nav configs################
#launch files
ADD configs/localization_launch.py /opt/ros/jazzy/share/robot_localization/launch/
ADD configs/navigation_launch.py /opt/ros/jazzy/share/nav2_bringup/launch/
ADD configs/navsat_transform.launch.py /opt/ros/jazzy/share/robot_localization/launch/
#configs
ADD configs/nav2_params.yaml /opt/ros/jazzy/share/nav2_bringup/params/
ADD configs/navsat_transform.yaml /opt/ros/jazzy/share/robot_localization/params/
##########W#A#Y#P#O#I#N#T#######################
#ADD configs/waypoints.yaml /opt/ros/jazzy/share/
#################################################
