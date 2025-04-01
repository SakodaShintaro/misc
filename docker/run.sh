#!/bin/bash
set -eux

cd $(dirname $0)

IMAGE_NAME=${1}
CONTAINER_NAME=${2}

docker build \
    --build-arg USER_NAME=$(whoami) \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    -t ${IMAGE_NAME} .

docker run -it \
    --name ${CONTAINER_NAME} \
    --user $(id -u):$(id -g) \
    --gpus all \
    --ipc=host \
    --network=host \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=$HOME/work:$HOME/work \
    --volume=$HOME/data:$HOME/data \
    --volume=$HOME/misc:$HOME/misc \
    --volume=$HOME/autoware:$HOME/autoware \
    --volume $HOME/.cache/:$HOME/.cache/ \
    --volume /media:/media:rw,shared \
    ${IMAGE_NAME} bash
