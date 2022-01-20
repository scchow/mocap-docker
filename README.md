# Using Docker for Mocap

Here is a DockerFile for setting up the ROS Driver on a machine that using a Docker container.
This will ideally allow for greater flexibility and portability.


# Prerequisites

Please install Docker by following the instructions [here](https://docs.docker.com/engine/install/)
Note if this your first time using Docker, you may need to follow the [post-install instructions](https://docs.docker.com/engine/install/linux-postinstall/) to run Docker as a non-root user.


# Creating an Image and Container

To create a Docker Image from the Dockerfile:

```
export DOCKER_BUILDKIT=1
docker build --ssh default -t mocap:base .
```

This will create a Docker Image and tag it `mocap:base`.


Next to create a container from the image, run:
```
docker container run -it --name dev --gpus all --network="host" mocap:base 
```

Notes:

I have not yet run this container on the actual mocap system, so some of the options: `--gpus all` and `--network="host"` may not be necessary. I will do more testing and update these instructions.