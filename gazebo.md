# Gazebo Installation Tutorials

## **RBE 500 Fall**

## *Made by Prof. Berk Calli and Yash Patil*

In this tutorial we will install Gazebo 11 and all its supporting files that will aid us in robot simulation.

## Gazebo

Use the following commands to install Gazebo 11 and all its supplementry files

```sh
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Once this is done, update using and make sure it runs without any errors

```sh
sudo apt-get update
```

now install Gazebo using

```sh
sudo apt-get install gazebo libgazebo-dev
```

You can check  your installation by running this in a new terminal

```sh
gazebo
```

## Gazebo ROS 2 packages

To use Gazebo with ros2, there are certain packages that need to be installed

```sh
sudo apt install ros-humble-gazebo-ros-pkgs
```

Now we will install simulation robot controllers

```sh
sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt-get install ros-humble-gazebo-ros2-control ros-humble-xacro
sudo apt-get install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
```

Now your system is ready to simulate any robot.

## Spawn rrbot in Gazebo

These are optional steps just to make sure everything is alright.