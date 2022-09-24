#!/bin/sh

ros2 topic pub --once euler_to_quat_topic std_msgs/msg/Float32MultiArray "{data: [$1, $2, $3]}"