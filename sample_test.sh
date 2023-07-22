#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# ノードをバックグラウンドで起動
ros2 launch my_package my_node.launch.py &

# レコードをバックグラウンドで起動
ros2 bag record -a -o output.bag &

# バッグファイルのリプレイ（これはフォアグラウンドで実行し、終了すると次の行に進む）
ros2 bag play input.bag

# すべてのバックグラウンドプロセスを停止
kill 0

# ここでoutput.bagを適当にパースして評価値を出力する
python3 parse_rosbag_output.py

# 秒数表示
echo "${SECONDS} sec"
