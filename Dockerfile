FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

# install apt dependencies and commonly used tools
# then remove apt lists to keep layer size down
# (https://stackoverflow.com/questions/61990329/dockerfile-benefits-of-repeated-apt-cache-cleans)
RUN apt update && \
    apt install -yq --no-install-recommends \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential \
        git \
        python3-catkin-tools \
        openssh-client \
        wget \
        vim \
        iputils-ping \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

ENV WORK_DIR=/home/catkin_ws
WORKDIR ${WORK_DIR}

# clone dependencies within container
RUN mkdir src && \
    cd src && \
    git clone https://github.com/scchow/motion_capture_system.git && \
    git clone https://github.com/scchow/qualisys_python_sdk.git && \
    git clone https://github.com/scchow/qualisys_cpp_sdk.git

# build ROS package
RUN source /opt/ros/${ROS_DISTRO}/setup.bash &&\
    catkin build

# build non-ROS CPP SDK (that also lives in src)
# we could use CMAKE_INSTALL_PREFIX to install to catkin_ws/install,
# which would put all of our executables in one folder make it easier
# to create a multi-stage Docker build with a tiny image.
RUN cd src/qualisys_cpp_sdk && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_EXAMPLES=ON && \
    cmake --build .

# install dependency for qualysis_python_sdk
RUN python3 -m pip install qtm

COPY entrypoint.bash ./entrypoint.bash
ENTRYPOINT ["/bin/bash", "-c", "source ${WORK_DIR}/entrypoint.bash && \"$@\"", "-s"]
CMD ["bash"]