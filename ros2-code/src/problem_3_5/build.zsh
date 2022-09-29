#!/bin/zsh

echo "Installing dependencies..."
rosdep install -i --from-path src --rosdistro humble -y
echo "Building euler_to_quat..."
colcon build --packages-select problem_3_5
echo "Done!"