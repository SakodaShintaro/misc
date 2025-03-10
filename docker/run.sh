#!/bin/bash
set -eux

cd $(dirname $0)

IMAGE_NAME=${1}

docker build \
    --build-arg USER_NAME=$(whoami) \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    -t ${IMAGE_NAME} .

docker run -it \
    --user $(id -u):$(id -g) \
    --gpus all \
    --ipc=host \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=$HOME/work:$HOME/work \
    --volume=$HOME/data:$HOME/data \
    --volume $HOME/.cache/:$HOME/.cache/ \
    --volume /media:/media:rwx,shared \
    ${IMAGE_NAME} bash
