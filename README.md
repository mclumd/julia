# Julia Demo

## Installation
Ensure the Baxter and Workstation are setup as described here: http://sdk.rethinkrobotics.com/wiki/Workstation_Setup

For Ubuntu 14.04:
```
sudo apt-get install ros-indigo-sound-play
sudo apt-get install ros-indigo-pocketsphinx
sudo apt-get install gstreamer0.10-pocketsphinx
```
Ensure system audio settings have the correct speakers and microphone chosen.

Clone this repo into `~/ros_ws/src` and run `catkin_make`

## Running
If everything is setup acccording to this guide: http://sdk.rethinkrobotics.com/wiki/Workstation_Setup
```
cd ~/ros_ws
. baxter.sh
roslaunch julia demo.launch
```


## Possible Issues
Make sure you:
```
source /opt/ros/indigo/setup.bash
source ~/ros_ws/devel/setup.bash
```
or add them to your `~/.bashrc`
