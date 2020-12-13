This directory contains code to be run on a computer that allows one to extract information
from the RPi.

## Installation requirements:
- Install OpenCV2 (`sudo apt-get install python-opencv`)
- Install ROS Melodic (follow [these](http://wiki.ros.org/melodic/Installation/Ubuntu)
  instructions). Make sure to follow all steps.

To build the workspace:
- In this directory, run `catkin_make`.
- Source `./devel/setup.bash`. Add this to your `~/.bashrc` for convience.

## Setting up communication between RPi and Laptop

See the [RPi's readme for instructions](../../rosnetwork_master/catkin_ws/README.md#Setting-up-communication-between-RPi-and-Laptop) to configure
the laptop's network settings correctly.
