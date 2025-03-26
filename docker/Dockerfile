FROM osrf/ros:jazzy-desktop-full

RUN apt update && apt upgrade -y
RUN apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
WORKDIR /workspace
ENV SHELL=/bin/[bash]
ENV ROS_DOMAIN_ID=2
# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************