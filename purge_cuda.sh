#!/bin/bash
set -eux

sudo apt-mark unhold cuda-*
sudo apt-mark unhold nvidia-*
sudo apt-mark unhold libcudnn*
sudo apt-mark unhold libnv*

sudo apt purge -y cuda-*
sudo apt purge -y nvidia-*
sudo apt purge -y libcudnn*
sudo apt purge -y libnv*
