#!/bin/zsh

echo "Installing dependencies..."
rosdep install -i --from-path src --rosdistro humble -y
echo "Building euler_to_quat..."
colcon build --packages-select euler_to_quat
echo "Installing euler_to_quat..."
source install/setup.zsh
echo "Done!"