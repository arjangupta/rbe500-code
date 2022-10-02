#!/bin/sh

ros2 topic pub --once inverse_kinematics_topic geometry_msgs/msg/Pose "{position: {x: $1, y: $2, z: $3}, orientation: {x: $4, y: $5, z: $6, w: $7}}"