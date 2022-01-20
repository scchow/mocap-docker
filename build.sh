# Creates a docker image from the current directory

# Usage:
# ./create_image.sh <image_name:tag>
# e.g., ./create_image.sh mocap:base

docker build -t $1 .