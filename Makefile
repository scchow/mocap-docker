DOCKER_IMAGE=mocap
DOCKER_IMAGE_REMOTE=playertr/mocap


#
# Tasks for building the ROS server and cpp client dockers images
#
build-image: Dockerfile entrypoint.bash
	docker build . -f Dockerfile -t ${DOCKER_IMAGE}

pull:
	docker pull ${DOCKER_IMAGE_REMOTE} && docker tag ${DOCKER_IMAGE_REMOTE} ${DOCKER_IMAGE}


#
# Tasks to run the ROS server, and cpp client images
#
run-ros:
	docker run -it --net host ${DOCKER_IMAGE} roslaunch mocap_qualisys qualisys.launch

run-cpp-sdk:
	docker run -it --net host ${DOCKER_IMAGE} ./src/qualisys_cpp_sdk/build/RigidBodyStreaming

run-py-sdk:
	docker run -it --net host ${DOCKER_IMAGE} python3 src/qualisys_python_sdk/examples/basic_example.py


.phony: build-image pull run-ros run-cpp-sdk run-py-sdk