#!/bin/zsh

echo "Installing dependencies..."
rosdep install -i --from-path src --rosdistro humble -y
echo "Building package problem_3_5..."
colcon build --packages-select problem_3_5
echo "Done!"