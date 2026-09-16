# RRT_Cut

ROS 1 motion-planning project with a Docker environment based on Ubuntu 20.04 and ROS Noetic. It can be developed from an Ubuntu 22.04 host through Docker.

## Prerequisites

- Git
- Docker Engine (recommended) or Docker Desktop for Linux
- An X11 desktop session, for ROS GUI applications such as RViz

Check that Docker is running before building:

```bash
docker info
```

If this command cannot connect to the Docker daemon, start Docker first. With Docker Engine use `sudo systemctl start docker`; with Docker Desktop use `systemctl --user start docker-desktop`.

## Run with Docker

1. Clone the repository and enter it.

   ```bash
   git clone https://github.com/minhtu0912/ros_motion_planning_tu.git
   cd RRT_Cut
   ```

2. Build the image.

   ```bash
   chmod +x docker/build.sh docker/run.sh
   ./docker/build.sh
   ```

   This creates the `ros_motion_planning:noetic` image.

3. Start the development container.

   ```bash

   systemctl --user start docker-desktop
    docker desktop status
    docker context use desktop-linux
   ./docker/run.sh
   ```

   The script grants the container access to the host X11 display and mounts this repository at `/root/ros_motion_planning_ws`. The container is interactive and is removed when you exit it.

4. Build the ROS workspace inside the container.

   ```bash
   cd /root/ros_motion_planning_ws
   source /opt/ros/noetic/setup.bash
   catkin_make
   source devel/setup.bash
   ```

5. Run the project inside the container.

   ```bash
   cd scripts
   ./build.sh
   ./main.sh
   ```

## Docker Desktop and X11

`docker/run.sh` mounts `/tmp/.X11-unix` so GUI applications in the container can use the host display. Docker Desktop runs containers in a VM and may reject this mount with a `mounts denied` error.

In Docker Desktop, open **Settings → Resources → File Sharing**, add the following paths, then choose **Apply & restart**:

```text
/tmp/.X11-unix
/home/<your-user>/RRT_Cut
```

Then retry `./docker/run.sh`. Docker Engine installed directly on Ubuntu is recommended for this project because it uses host networking, privileged mode, and X11 GUI forwarding.
