#!/bin/bash

TIMESTAMP=$(date +%Y-%m-%d_%H-%M)

ros2 bag record -o "/home/ubuntu/ros2_ws/src/ImMesh/Log/path_${TIMESTAMP}" --topics /path