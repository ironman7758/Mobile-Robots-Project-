# create workspace
mkdir -p ~/camera_ws/src
cd ~/camera_ws/src

# check out libcamera
sudo apt -y install python3-colcon-meson
# Option A: official upstream
git clone https://git.libcamera.org/libcamera/libcamera.git
# Option B: raspberrypi fork with support for newer camera modules
# git clone https://github.com/raspberrypi/libcamera.git

# check out this camera_ros repository
git clone https://github.com/christianrauch/camera_ros.git

# resolve binary dependencies and build workspace
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/camera_ws/
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
colcon build --event-handlers=console_direct+

