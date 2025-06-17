#!/bin/bash
set -eux

sudo apt-mark unhold cuda-* nvidia-* libcudnn* libnv*
sudo apt purge -y cuda-* nvidia-* libcudnn* libnv*

# 確認
# ls -la /etc/alternatives/cuda
# find /usr/lib/x86_64-linux-gnu/ -name "libcudnn.so*"
# find /usr/lib/x86_64-linux-gnu/ -name "libnvinfer.so*"
