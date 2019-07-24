# Julia Demo

## Installation
Ensure the Baxter and Workstation are setup as described [here](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).

For Ubuntu 14.04:
```
sudo apt-get install ros-indigo-sound-play
sudo apt-get install ros-indigo-pocketsphinx
sudo apt-get install gstreamer0.10-pocketsphinx
```
Ensure system audio settings have the correct speakers and microphone chosen.

Clone this repo into `~/ros_ws/src` and run `catkin_make`

## Running
If everything is setup acccording to [this guide](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup):
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
Download [yolov3.weights](https://pjreddie.com/media/files/yolov3.weights) into the `/scripts` directory.

Gaze Detection is done with a modified version of [this repo](https://github.com/antoinelame/GazeTracking). You will need to `sudo pip install dlib opencv-python numpy`. Be sure to use OpenCV4, not 3 as it says in the linked repo.

#### ALMA
[ALMA](https://github.com/mclumd/alma-2.0) is included as a submodule. After initializing, you will need to run `make` in the `/alma` directory.
