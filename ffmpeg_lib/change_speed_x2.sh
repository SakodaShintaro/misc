#!/bin/bash
set -eux

INPUT_PATH=$(readlink -f $1)
OUTPUT_PATH=${INPUT_PATH%.*}_x2.${INPUT_PATH##*.}

ffmpeg -i ${INPUT_PATH} \
    -vf "setpts=0.5*PTS" \
    -af "atempo=2.0" \
    -c:v libvpx-vp9 \
    -speed 4 \
    -c:a libvorbis \
    ${OUTPUT_PATH}
