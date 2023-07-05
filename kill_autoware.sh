#!/bin/bash

pkill ros2
pkill rviz2
pkill aggregator_node
ps aux | grep python3 | grep ros2 | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep python3 | grep rqt_reconfigure | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep component_container | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep robot_state_publisher | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep topic_tools/relay | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep "ros-args" | grep -v grep | awk '{ print "kill ", $2 }' | sh
ps aux | grep "AWSIM_demo.x86_64" | grep -v grep | awk '{ print "kill ", $2 }' | sh

ros2 daemon stop
ros2 daemon start
