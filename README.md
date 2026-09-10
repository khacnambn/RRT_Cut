RRT_Cut
ROS Motion Planning TU
This repository contains a ROS1 motion planning project with Docker support.

The Docker environment is based on Ubuntu 20.04 and ROS Noetic, so this project can also be run on a host machine using Ubuntu 22.04 through Docker.

Repository structure
ros_motion_planning_tu/
├── 3rd/
├── assets/
├── docker/
│   ├── Dockerfile
│   ├── build.sh
│   └── run.sh
├── docs/
├── scripts/
├── src/
├── .dockerignore
├── .gitignore
└── README.md

```bash
1. Clone repository
git clone https://github.com/minhtu0912/ros_motion_planning_tu.git
cd RRT_Cut

2. Build Docker image
chmod +x docker/build.sh
./docker/build.sh

3. Run Docker container
chmod +x docker/run.sh
./docker/run.sh

This command opens the Docker container and mounts the project into:

/root/ros_motion_planning_ws

4. Build ROS1 workspace inside Docker

Inside the Docker container, run:

cd /root/ros_motion_planning_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

5. Run Launch File 
cd scripts/
./build.sh 
./main.sh
```
