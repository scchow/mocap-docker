# Using Docker for Mocap

Here is a DockerFile for setting up Real Time motion capture for Qualisys on a machine that using a Docker container.
This will ideally allow for greater flexibility and portability.

The scripts in this repository:
- Clone the relevant repositories into a folder
- Create a Docker image with ROS Noetic and dependencies installed
- Create a Docker container that mounts the folder with relevant repositories for easy building and development


## Prerequisites

Please install Docker by following the instructions [here](https://docs.docker.com/engine/install/)

Note: if this your first time using Docker, you may need to follow the [post-install instructions](https://docs.docker.com/engine/install/linux-postinstall/) to run Docker as a non-root user.


## Installation Instructions

We provide two ways to build the Docker images/containers. 

We provide a series of bash scripts that simplify some of the interface with Docker. These scripts are mostly for convenience, but can also be used by those unfamiliar with Docker.

The Dockerfile can also be used to create images and containers with standard Docker build commands.

### Bash Scripts
We have provided a series of bash scripts that can build the image, create a container, and run/attach to containers to simplify operations. They work as follows:

```bash
bash clone_dependencies.sh # clones the repositories for mocap
bash build.sh <image_name>:<image_tag> # invokes Docker to build the image according to the DockerFile
bash run.sh <image_name>:<image_tag> <container_name> # Uses the image to build a container and starts it
bash start.sh <container_name> # starts the given Docker container
bash attach.sh <container_name> # attach to a running Docker container
```

#### Bash Quickstart

Run the following commands to construct an image, create a container, and run the container, dropping into a terminal.

``` bash
bash clone_dependencies.sh
bash build.sh mocap:base
bash run.sh mocap:base mocap-dev
```

Once you've closed the container and stopped it, you can start it by running
``` bash
bash start.sh mocap-dev
bash attach.sh mocap-dev # run this in as many terminals as you want for multiple views into a container
```

### Manual: Creating an Image and Container

To create a Docker Image from the Dockerfile:

``` bash
docker build -t mocap:base .
```

This will create a Docker Image and tag it `mocap:base`.


Next to create a container from the image, run:
``` bash
docker container run -it --name mocap-dev --network="host" mocap:base 
```

You can now restart/attach to that container using:

``` bash
docker container start mocap-dev
docker container attach mocap-dev
```

Notes:

I have not yet run this container on the actual mocap system, so some of the options: `--network="host"` may not be necessary. I will do more testing and update these instructions.


## Things to do inside the Docker container

Once you have gotten the Docker container up and running, you can now start building and installing packages.

### ROS Motion Capture Package
The ROS packages can be built and run by:
``` bash
cd /home/catkin_ws # If you aren't there already
catkin build
source devel/setup.bash # Remember to source devel/setup.bash every time you restart the container
roslaunch mocap_qualisys qualisys.launch
```

### C++ Motion Capture Package
The C++ SDK packages can be built and run by running:

``` bash
cd /home/catkin_ws/src/qualisys_cpp_sdk
mkdir build
cd build
cmake .. -DBUILD_EXAMPLES=ON
cmake --build .
./RigidBodyStreaming
```

### Python Motion Capture Package

The Python SDK packages are installed and ran by:

```bash
cd /home/catkin_ws/src/qualisys_python_sdk
python3 -m pip install qtm
python3 examples/basic_example.py
```
