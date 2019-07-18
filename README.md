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
Note that to run anything in the `scripts` directory using `rosrun`, you will need to make the python files executable.
When writing new python files, `#!/usr/bin/env python` must be included at the beginning of the file.


## Possible Issues
Make sure you:
```
source /opt/ros/indigo/setup.bash
source ~/ros_ws/devel/setup.bash
```
or add them to your `~/.bashrc`

## Included Modules

#### Vision
Read the documentation for [YOLO](https://github.com/mclumd/yolo) to run the detection script.

#### ALMA
You might need to rebuild [ALMA](https://github.com/mclumd/alma-2.0) if you modify the `alma.c` file.
