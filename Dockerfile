FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]
RUN apt update && apt upgrade -y
RUN apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git python3-catkin-tools
# Install some commonly used tools
RUN apt-get install -y openssh-client wget vim iputils-ping python3-pip

# RUN source /opt/ros/noetic/setup.bash && catkin build
# RUN sed -i '$i\source "/home/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

# RUN mkdir -p /home/.ssh
# RUN echo "Host *.trabe.io\n\tStrictHostKeyChecking no\n" >> /root/.ssh/config

WORKDIR /home/catkin_ws
RUN mkdir src

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]