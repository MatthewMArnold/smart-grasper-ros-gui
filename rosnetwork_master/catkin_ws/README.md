This directory contains code for running on a Raspberry Pi as a ROS master.

## Platform requirements:
- Raspberry Pi 4.0
- Ubuntu 18.04 LST operating system flashed to the RPi SD card (use
  [this](https://github.com/TheRemote/Ubuntu-Server-raspi4-unofficial/releases)
  distribution)

## Installation requirements:
- Install OpenCV2 (`sudo apt-get install python-opencv`)
- Install ROS Melodic (follow [these](http://wiki.ros.org/melodic/Installation/Ubuntu)
  instructions). Make sure to follow all steps.
- Enable UART on the RPi (see [the official instructions](https://www.raspberrypi.org/documentation/configuration/uart.md)
  or [these instructions](https://raspberrypi.stackexchange.com/questions/114366/rpi4-serial-port-not-working-on-either-raspberry-os-or-ubuntu)
  that I found helpful). Be sure to `sudo apt install python-serial`.
- Install qt5 and associated packages:
  ```
  sudo apt-install qtcreator
  sudo apt-install qt5
  sudo apt install qml-module-qtquick-controls2
  sudo apt install libqt5charts5 libqt5charts5-dev
  ```
- Install [this](https://github.com/severin-lemaignan/ros-qml-plugin) software
  to allow you to run ROS in a qt project.

To build the workspace:
- In this directory, run `catkin_make`.
- Source `./devel/setup.bash`. Add this to your `~/.bashrc` for convience.

To run the ROS network (still a WIP):
```bash
./start_network.sh
```

## Setting up communication between RPi and Laptop

The RPi is the ROS master. To configure the network properly, you first must set
up a static IP address for the RPi (see
[here](https://computingforgeeks.com/how-to-configure-static-ip-address-on-ubuntu/)
if you are unfamiliar with setting this up). You must do this for both the RPi
and the laptop.

Next disable the firewall on the RPi and laptop (`sudo ufw disable`). To see if you
can communicate with the RPi and laptop, ping both devices.

On the RPi, open `~/.bashrc` and add the following:
```bash
export ROS_MASTER_URI=http://localhost:11311/
export ROS_HOSTNAME=$RPI_IP_ADDR
export ROS_IP=$RPI_IP_ADDR
```
where RPI_IP_ADDR is the static ip address of the RPi you configured earlier.

Now, on the laptop, open `~/.bashrc` and add the following:
```bash
export ROS_MASTER_URI=http://$LAPTOP_IP_ADDR:11311/
export ROS_HOSTNAME=$RPI_IP_ADDR
export ROS_IP=$RPI_IP_ADDR
```
where LAPTOP_IP_ADDR is the static ip address of the laptop you configured earlier.
Note that the only thing different from the RPi is the first line.

## Synchronizing RPi's slock with laptop

To synchronize the RPi4's clock with the laptop, `sudo apt-get install chrony` and
then add this line to `/etc/chrony/chrony.conf`:
```
server <c1> minpoll 0 maxpoll 5 maxdelay .05
```
Where `<cl>` is the name of the laptop

## Modifying the GUI

The GUI is written using qt4 because ROS doesn't fully support qt5. The GUI is
run as a ROS plugin.

### Modifying the layout

To modify the UI, I use qtcreator. The `.ui` files in `src/gui_pkg/resource`
define the types and orientation of widgets. To open these files in qtcreator,
open qtcreator (`qtcreator` in terminal). In the qtcreator UI, click "File",
then "Open File or Project." Navigate to `src/gui_pkg/resource` in qtcreator and
select the file you would like to modify.

### Modifying content

TODO
