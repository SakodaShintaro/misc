#!/bin/bash
set -eux

sudo apt update
sudo apt -y upgrade
sudo apt -y autoremove
