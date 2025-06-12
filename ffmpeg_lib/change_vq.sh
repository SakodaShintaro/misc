#!/bin/bash
set -eux

INPUT_PATH=$(readlink -f "$1")
OUTPUT_PATH=${INPUT_PATH%.*}_reduced_x2.${INPUT_PATH##*.}

ffmpeg -i "${INPUT_PATH}" \
    -vf "scale=1280:-1,fps=24,setpts=0.5*PTS" \
    -af "atempo=2.0" \
    -b:v 500k \
    -c:v libvpx-vp9 \
    -speed 4 \
    -c:a libvorbis \
    "${OUTPUT_PATH}"
