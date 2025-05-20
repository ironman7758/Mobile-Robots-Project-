-- build container
docker build -t <name> .
docker compose build
docker compose up

--run and attach on terminal
docker exec -it <name> /bin/bash

-- source ros if lost
source /opt/ros/jazzy/setup.bash

-- joy will run automatically
-- to run anything else do:
ros2 run <packagename> <nodename>
--specifally:
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps4' joy_vel:="joy_vel"
ros2 run ariaNode ariaNode -rp /dev/ttyUSB0


--launching phigets spatial
 ros2 launch phidgets_spatial spatial-launch.py


 ---Extra to make sure display works!:
 export DISPLAY=:0.0


 -----------


 # In your ros2 workspace
cd ~/ros_ws

# Build and source
colcon build
source install/setup.bash

# Add your package’s share directory to Gazebo’s resource path
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix p3at_description)/share
echo $GZ_SIM_RESOURCE_PATH

# Add your models directory to Gazebo’s model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/gazebo_ws-jazzy/src/p3at_description/models
echo $GAZEBO_MODEL_PATH

# Install some ROS 2 Jazzy packages
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-diagnostic-updater

# Launch your robot description in RViz/Gazebo
ros2 launch p3at_description display.launch.py

# Teleoperate the robot with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard


ros2 run tf2_tools view_frames