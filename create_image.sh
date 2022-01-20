# Creates a docker image from the current directory

# Usage:
# ./create_image.sh <image_name:tag>
# e.g., ./create_image.sh mocap:base

export DOCKER_BUILDKIT=1
docker build --ssh default -t $1 .