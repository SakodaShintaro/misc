#!/bin/bash
set -eux

INPUT_PATH=$(readlink -f $1)
OUTPUT_PATH=${INPUT_PATH%.*}.gif

ffmpeg -i ${INPUT_PATH} \
    -filter_complex "[0:v] fps=5,scale=320:-1,split [a][b];[a] palettegen=max_colors=64 [p];[b][p] paletteuse" \
    ${OUTPUT_PATH}
