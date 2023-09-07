#!/bin/bash
set -eux

INPUT_PATH=$(readlink -f $1)
OUTPUT_PATH=${INPUT_PATH%.*}_reduced.${INPUT_PATH##*.}

# ビットレートを500k、フレームレートを24fps、解像度を1280x-1に設定
ffmpeg -i ${INPUT_PATH} \
    -vf "scale=1280:-1,fps=24" \
    -b:v 500k \
    -c:v libvpx-vp9 \
    -c:a libvorbis \
    ${OUTPUT_PATH}
