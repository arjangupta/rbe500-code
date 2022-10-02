#!/bin/zsh

echo "Installing dependencies..."
rosdep install -i --from-path src --rosdistro humble -y
echo "Building package homework4_ros..."
colcon build --packages-select homework4_ros
echo "Done!"