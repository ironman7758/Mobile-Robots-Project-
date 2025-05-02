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