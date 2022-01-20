# Attaches to a running Docker container
#
# Usage:
# ./attach_docker.sh <container_name>
# e.g. ./run_docker.sh mocap-dev

docker container attach $1 