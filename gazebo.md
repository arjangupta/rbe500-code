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

Please find the zipped package that has one of many ways to spawn an URDF in Gazebo using ROS2.

Extract the package to your workspace/src.

Before doing colon build we have to install some dependencies for the package to function correctly.

Run the following commands in terminal.

```sh
sudo apt install python3-pip
sudo pip3 install transforms3d
```

Once you have successfully installed all the required packages, find the supporting zip file that contains rrbot simulation files.

## RRBOT

That zip file contains 2 packages
**rrbot description**
**rrbot gazebo**

### rrbot_description

This package contains the description files for the rrbot such as its urdf, gazebo plugins, ros2 control definitions, etc.
Xacro has been used instead of URDF to keep the description more modular, you will find that the main file "rrbot.urdf.xacro" includes a bunch of different files, so you don't have to deal with a very large file.

The important files for us are "rrbot.gazebo.xacro" and "rrbot.ros2_control.xacro", these contains the configuration for ROS2 Control.

-------skipped text-------

```sh
colcon build --symlink-install
. install/setup.bash
```

```sh
. /usr/share/gazebo/setup.bash
```
