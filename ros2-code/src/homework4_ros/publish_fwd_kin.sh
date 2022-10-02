#!/bin/sh

ros2 topic pub --once forward_kinematics_topic std_msgs/msg/Float32MultiArray "{data: [$1, $2, $3]}"