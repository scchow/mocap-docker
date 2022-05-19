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

When cloning this repository, please use 
```
git clone --recurse-submodules git@github.com:scchow/mocap-docker.git
```
to also clone dependencies as submodules.


## Installation Instructions

We provide two ways to build the Docker images/containers. 

We provide a series of bash scripts that simplify some of the interface with Docker. These scripts are mostly for convenience, but can also be used by those unfamiliar with Docker.

The Dockerfile can also be used to create images and containers with standard Docker build commands.

### Make Scripts
We have provided a series of bash script and a Makefile that can build the image, create a container, and run/attach to containers to simplify operations. They work as follows:

```bash
make build <image_name>:<image_tag> # invokes Docker to build the image according to the DockerFile
make run <image_name>:<image_tag> <container_name> # Uses the image to build a container and starts it
make start <container_name> # starts the given Docker container
make attach <container_name> # attach to a running Docker container sharing tty session
make ssh <container_name> # create new bash tty into running container
```

### Bash Quickstart

Run the following commands to construct an image, create a container, and run the container, dropping into a terminal.

Note: you may omit the `mocap:base` `mocap-dev` from the following commands, since they are the default arguments for `image_name` and `container_name`. Alternatively, you may want to change the image and container names, in which case you would replace `mocap-base` and `mocap-dev` respectively.


``` bash
make build mocap:base
make run mocap:base mocap-dev
```

Once you've closed the container and stopped it, you can start it by running
``` bash
make start mocap-dev
make ssh mocap-dev # run this in as many terminals as you want for multiple views into a container
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


## Things to do inside the Docker container

Once you have gotten the Docker container up and running, you can now start building and installing packages.

### Setup Networking

On the main computer, the Qualisys software must be running and a 6DOF Rigid Body should be created by selecting at least 3 visible markers in the 3D view (by clicking each marker while holding ctrl), right-clicking and selecting `Add Rigid Object`.

To find the IP of the main computer, open up a Command Window and type `ifconfig`.
Take note of the `192.168.XXX.YYY` address.

Plug in the laptop or computer with the Docker container into the ethernet switch.

You must manually configure the network by doing the following:

  1. Go to network settings
  2. Change IPv4 to Manual
  3. Set address to 192.168.XXX.ZZ - Notice the XXX must match the IP of the main computer, ZZZ can be any unused address. 
  4. Set Netmask: 255.255.255.128
  5. Leave Gateway Blank
  6. Perform the edits to each of the SDK to ensure they can connect to the correct IP Address.

### ROS Motion Capture Package

The ROS packages can be built and run by:
``` bash
cd /home/catkin_ws # If you aren't there already
catkin build
source devel/setup.bash # Remember to source devel/setup.bash every time you restart the container
roslaunch mocap_qualisys qualisys.launch server_address:=192.168.XXX.YYY # Where XXX.YYY corresponds to IP address of computer running qualisys software
```

### C++ Motion Capture Package

For the example script `qualisys_cpp_sdk/RigidBodyStreaming/RigidBodyStreaming.cpp`, 
- Set `serverAddr[]` to `192.168.XXX.YYY` matching the Qualisys running machine.
- Examine the version of Qualisys RealtTime being run on the main machine. (Ours is 1.22.)
- Sset `majorVersion=1` and `minorVersion=22` to match the version number


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

For the basic example script `qualisys_python_sdk/examples/basic_example.py`,
update the `qtm.connect()` function call to the Qualisys machine's IP address, the version number of the Real-Time connect software running on the desktop and the port number (e.g., `qtm.connect("192.168.XXX.YY", version="1.22", port=22223)`).
Note that Python connects over port 22223 instead of 22222 used by the ROS/C++ APIs

The Python SDK packages are installed and ran by:
```bash
cd /home/catkin_ws/src/qualisys_python_sdk
python3 -m pip install qtm
python3 examples/basic_example.py
```

For `stream_dof_example.py`, you must preform the same change as above, set the `realtime` variable to be `True`, and input the name of the body to be tracked in `wanted_body`. Additionally, you must set the amount of time to monitor with in the line `await asyncio.sleep(10)` where 10 is replaced by the number of seconds to monitor.

## Notes

- It does not seem like the Qualisys software on their computer needs to be actively recording to run!
- The above process works outside the Docker container as well, as long as you as you install the correct dependencies.
- If the mocap loses track of the object:
  - Python API: Returns Nans
  - C++ : returns Nans
  - ROS: Ignores in main topics and prints a rosinfo warn message

The `stream_dof_example.py` script is very rudimentary, only tracking one object at a time for a fixed number of seconds. This can be improved by integrating parts from `basic_example.py` to allow fo continuous running until `Keyboard Interrupt` and tracking multiple moving bodies.

Things to investigate
  a. Does the reported time in ROS message sync with the time in Qualisys?
  b. Does tracking moving targets work
  c. improve command line interface in scripts to allow easy editing of ip/port/version/etc.
  d. Debug extra print statement on dropped frames.

