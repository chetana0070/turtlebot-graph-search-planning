#!/bin/bash

# Execute this script to kill all processes related to turtlebot_search!
#
# WARNING: This will kill any process with turtlebot_search in the string. We hope that its
# only CSE471 related, but if you suspect otherwise then do NOT run this script.
pkill -f "ros*" --signal sigkill
killall gzserver
killall gzclient
pkill -f "python .*turtlebot_search.*" --signal sigkill
pkill -f roscore
