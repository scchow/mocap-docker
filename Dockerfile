FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]
RUN apt update && apt upgrade -y
RUN apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git python3-catkin-tools
RUN apt-get install -y openssh-client

WORKDIR /home/catkin_ws/src
# Use user's ssh key at build time: http://blog.oddbit.com/post/2019-02-24-docker-build-learns-about-secr/

# This is necessary to prevent the "git clone" operation from failing
# with an "unknown host key" error.
RUN mkdir /root/.ssh; \
  touch -m /root/.ssh/known_hosts; \
  ssh-keyscan github.com > /root/.ssh/known_hosts

RUN --mount=type=ssh git clone git@github.com:scchow/motion_capture_system.git
RUN --mount=type=ssh git clone git@github.com:scchow/qualisys_python_sdk.git
RUN --mount=type=ssh git clone git@github.com:scchow/qualisys_cpp_sdk.git

WORKDIR /home/catkin_ws
RUN source /opt/ros/noetic/setup.bash && catkin build
RUN sed -i '$i\source "/home/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

# RUN mkdir -p /home/.ssh
RUN echo "Host *.trabe.io\n\tStrictHostKeyChecking no\n" >> /root/.ssh/config

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]