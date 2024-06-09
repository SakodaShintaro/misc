#!/bin/bash

set -eux

TARGET_DIR=$1

# もとの連番前提のコマンド
# ffmpeg -r 10 \
#        -i ${TARGET_DIR}/%08d.png \
#        -vcodec libx264 \
#        -pix_fmt yuv420p \
#        -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" \
#        -r 10 \
#        ${TARGET_DIR}/../movie.mp4

# 連番ではないpngファイルらをmp4にする（順番はソートする）
file_list=$(find ${TARGET_DIR} -name "*.png" | sort -V)
ffmpeg -r 10 \
       -f concat -safe 0 -i <(printf "file '%s'\n" ${file_list}) \
       -vcodec libx264 \
       -pix_fmt yuv420p \
       -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" \
       -r 10 \
       ${TARGET_DIR}/../movie.mp4
