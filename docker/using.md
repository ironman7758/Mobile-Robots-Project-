-- build container
docker build -t <name> .
docker compose build
docker compose up

--run and attach on terminal
docker exec -it <name> /bin/bash

-- source ros if lost
source /opt/ros/jazzy/setup.bash

