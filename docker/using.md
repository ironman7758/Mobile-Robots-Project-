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


# SLAM SETUP + MANUAL DRIVE: or Full setup for part 2

# RUNNING CONTROLLER

cd /workspace/ros_ws
source install/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps4' joy_vel:="joy_vel"     

 # RUNNING ariaNode
cd /workspace/ros_ws
source install/setup.bash
ros2 run ariaNode ariaNode -rp /dev/ttyUSB0                  

 # Running master node,  connect contoller 
cd /workspace/ros_ws
source install/setup.bash
ros2 run master master_node                          

# LIDAR SETUP

cd richbeam/
source install/setup.bash
ros2 launch lakibeam1 lakibeam1_scan.launch.py                  


# Running Gazebo + topics

cd ~/ros_ws
colcon build
source install/setup.bash

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix p3at_description)/share   
# Add your package’s share directory to Gazebo’s resource path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/gazebo_ws-jazzy/src/p3at_description/models
# echo $GAZEBO_MODEL_PATH # Add your models directory to Gazebo’s model path

ros2 launch p3at_description display.launch.py rviz:=false use_sim_time:=false # Launch your robot description in RViz/Gazebo

# Navigation bringup

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true

# Online Async launch Slam_toolbox


ros2 launch slam_toolbox online_async_launch.py  slam_params_file:=/workspace/ros_ws/src/p3at_description/config/mapper_params_online_async.yaml use_sim_time:=false autostart:=true

# RVIZ

xhost +                                           # outside of container
export DISPLAY=: ...                              # (inside container)
rviz2

  ros2 run rviz2 rviz2 -d \
  $(ros2 pkg prefix p3at_description)/share/p3at_description/rviz/display.rviz


# CAMERA SETUP

# Run Number Detection Node:

  # republish node

  ros2 run image_transport republish raw raw \                      
    --ros-args \
      -r in:=/camera/rgb/image_raw \
      -r out:=/image
 
  # digit detection

    cd /workspace/ros_ws
    colcon build --packages-select digit_recognition           # 
    source install/setup.bash

    ros2 run digit_recognition digit_recognition_node \
      --ros-args \
        -r /camera/image_raw:=/image


# FOR Color Detection 

# Running color detection node
cd /workspace/ros_ws
colcon build --packages-select digit_recognition
source install/setup.bash

ros2 run digit_recognition color_detection_node


# Running color detection node and remap
cd /workspace/ros_ws
colcon build --packages-select digit_recognition
source install/setup.bash

ros2 run digit_recognition color_detection_node \
  --ros-args -r /camera/image_raw:=/image








# View tf tree
ros2 run tf2_tools view_frames

ros2 launch imu_filter_madgwick imu_filter.launch.py

--launching phigets spatial
 ros2 launch phidgets_spatial spatial-launch.py   #unsure 

 <!--   
#UPDATED (unsure)

ros2 run digit_recognition digit_recognition_node \
  --ros-args -r /camera/image_raw:=/image


3. in seperate terminals

ros2 run image_tools showimage --ros-args -r image:=/digit_thresh
ros2 run image_tools showimage --ros-args -r image:=/digit_annotated
ros2 topic echo /digit_classification_result -->