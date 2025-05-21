# SETUP

-- build container
docker build -t <name> .
docker compose build
docker compose up

--run and attach on terminal
docker exec -it <name> bash

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

# LIDAR SETUP


cd richbeam/
source install/setup.bash

ros2 launch lakibeam1 lakibeam1_scan.launch.py 



 # SLAMMING SETUP

cd ~/ros_ws
colcon build
source install/setup.bash

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix p3at_description)/share 
c  # Add your package’s share directory to Gazebo’s resource path


export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/gazebo_ws-jazzy/src/p3at_description/models
echo $GAZEBO_MODEL_PATH # Add your models directory to Gazebo’s model path



ros2 launch p3at_description display.launch.py rviz:=false use_sim_time:=false # Launch your robot description in RViz/Gazebo

Terminal 2:
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true

Terminal 3:
  ros2 run rviz2 rviz2 -d \
  $(ros2 pkg prefix p3at_description)/share/p3at_description/rviz/display.rviz


 ros2 launch slam_toolbox online_async_launch.py   slam_params_file:=/workspace/ros_ws/src/p3at_description/config/mapper_par
ams_online_async.yaml   use_sim_time:=false autostart:=true


  -------

  # CAMERA SETUP

Run Number Detection Node:

1. 
ros2 run image_transport republish raw raw \
  --ros-args \
    -r in:=/camera/rgb/image_raw \
    -r out:=/image


2. 

cd /workspace/ros_ws
colcon build --packages-select digit_recognition
source install/setup.bash

ros2 run digit_recognition digit_recognition_node \
  --ros-args \
    -r /camera/image_raw:=/image




3. in seperate terminals

ros2 run image_tools showimage --ros-args -r image:=/digit_thresh
ros2 run image_tools showimage --ros-args -r image:=/digit_annotated
ros2 topic echo /digit_classification_result





