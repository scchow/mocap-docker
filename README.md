> **_NOTE:_**
>
>   In this `deploy` branch, I've modified the code to follow different Docker conventions. The `master` branch is probably a better development environment because it bind-mounts the important code for easy edits. These are the changes I've made in this branch.
>
>   * I removed the `apt/lists` files after each installation command, to minimize layer size.
>   * I made `WORK_DIR` an environment variable.
>   * I baked all of the installation steps into the Dockerfile, so the user does not have to do them. This is both more convenient and more immutable/reproducible. It's still possible to bind-mount local versions into the container for local development.
>   * I removed the `ssh` key bind-mounts, since they are not strictly necessary.
>   * I added `source devel/setup.bash` into a custom entrypoint.
>   * I removed the helper bash scripts, preferring explicit Docker `build`/`pull`/`run` commands.
>   * I pushed an image to Docker Hub for easy access.
>   
>   A couple more changes might make this Dockerfile more idiomatic:
>   * use a multi-stage build to create a smaller image containing only the executables for deployment.
>   * make shorter aliases for each command so the `docker run` command can be shorter and not rely on the installation location.
>   * Bake separate Dockerfiles for each command, and organize them in a `docker_compose.yml` to form a true community of microservices.
>   * Create a non-root user, as a best practice for security.


# Using Docker for Mocap

Here is a DockerFile for setting up Real Time motion capture for Qualisys on a machine that using a Docker container.
This will ideally allow for greater flexibility and portability.

The scripts in this repository:
- Create a Docker image with ROS Noetic, dependencies, and relevant repositories installed

## Prerequisites

Please install Docker by following the instructions [here](https://docs.docker.com/engine/install/)

Note: if this your first time using Docker, you may need to follow the [post-install instructions](https://docs.docker.com/engine/install/linux-postinstall/) to run Docker as a non-root user.


## Installation Instructions

Build or pull the Docker image for MoCap:

Pull (preferred):
```
docker pull playertr/mocap
```

Build (alternative, use if you want to compile the dependencies locally):
```
docker build . -f Dockerfile
```

## Usage Instructions

The MoCap ROS nodes, C++ SDK, and python SDK can be run with the following commands, respectively:

```
docker run -it --net host playertr/mocap roslaunch mocap_qualisys qualisys.launch
```

```
docker run -it --net host playertr/mocap ./src/qualisys_cpp_sdk/build/RigidBodyStreaming
```

```
docker run -it --net host playertr/mocap python3 src/qualisys_python_sdk/examples/basic_example.py
```

We have additionally added `make` targets for these commands, so you can run `make run-ros`, `make run-cpp-sdk`, or `make run-py-sdk` if you are lazy or efficient.