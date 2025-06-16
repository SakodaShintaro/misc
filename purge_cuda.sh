#!/bin/bash
set -eux

sudo apt-mark unhold cuda-* nvidia-* libcudnn* libnv*
sudo apt purge -y cuda-* nvidia-* libcudnn* libnv*
