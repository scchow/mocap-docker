# Runs a Docker container
#
# Usage:
# ./run_docker.sh <image_name:tag> <container_name>
# e.g. ./run_docker.sh mocap:base mocap-dev

docker container run -it -v ~/.ssh:/root/.ssh:ro --name $2 --network="host" $1 