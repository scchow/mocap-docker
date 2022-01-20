# Runs a Docker container
#
# Usage:
# ./run.sh <image_name:tag> <container_name>
# e.g. ./run.sh mocap:base mocap-dev

docker container run -it \
-v ~/.ssh:/root/.ssh:ro \
-v $(pwd)/deps:/home/catkin_ws/src \
--name $2 \
--network="host" \
$1 