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

```
cd ~/ros_ws/src
git clone --recursive https://github.com/mclumd/julia.git
git submodule update --init
catkin_make
```


### Included Modules

#### Vision
Read the documentation for [YOLO](https://github.com/mclumd/yolo) to run the detection script.
Download [yolov3.weights](https://pjreddie.com/media/files/yolov3.weights) into the `/scripts` directory.

Gaze Detection is done with a modified version of [this repo](https://github.com/antoinelame/GazeTracking). You will need to `sudo pip install dlib opencv-python numpy`. Be sure to use OpenCV4, not 3 as it says in the linked repo.

#### ALMA
[ALMA](https://github.com/mclumd/alma-2.0) is included as a submodule. After initializing, you will need to run `make` in the `/alma` directory.

When pulling, to pull all changes in the repo including changes in the submodules:
`git pull --recurse-submodules`

To pull all changes for the submodules only,
`git submodule update --remote`

Or just: 
```
cd alma
git pull
make
```
## Running
Make sure you `source /opt/ros/indigo/setup.bash` and `source ~/ros_ws/devel/setup.bash` or add them to your `~/.bashrc`.

If everything is setup acccording to [this guide](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup):

```
cd ~/ros_ws
. baxter.sh
roslaunch julia demo.launch
# In a second terminal
cd ~/ros_ws
. baxter.sh
rosrun julia alma_bridge.py
```
With two terminals, the ALMA output is separate from the ROS output, though the demo can be run as one program if `alma_bridge.py` is added to the `demo.launch` file.

Note that to run anything in the `scripts` directory using `rosrun`, you will need to make the python files executable.
When writing new python files, `#!/usr/bin/env python` must be included at the beginning of the file.

## Troubleshooting

Sometimes pocketsphinx (speech recognizer) doesn't want to work. It will hang at 
```
INFO: ngram_search_fwdtree.c(186): Creating search tree
INFO: ngram_search_fwdtree.c(191): before: 0 root, 0 non-root channels, 12 single-phone words
INFO: ngram_search_fwdtree.c(326): after: max nonroot chan increased to 331
INFO: ngram_search_fwdtree.c(338): after: 78 root, 203 non-root channels, 11 single-phone words
```
Usually a `Ctrl-c` and running again will solve it, but sometimes it helps to run one of pocketsphinx's demos (like `roslaunch pocketsphinx robocup.launch`). You'll know it's working if you get a message like `[INFO] [WallTime: 1565101552.133054] the`, where "the" is the recognized speech.

Check the system settings to make sure the default mic and speakers are what you want them to be. Sometimes it will recognize if you start talking as soon as the program starts, but sometimes it only works if you stay silent while the program starts and wait for 5 seconds. Seems to be a pocketsphinx bug.


## Explanation

When it recognizes the word "Julia," it adds `hearing(julia)` to ALMA. 

At the time step ALMA receives `hearing(julia)`, if `talking` is not present in the knowledge base (meaning the Baxter isn't talking), and it has seen a person ("Julia"), it will point to the last place it saw that person, and say "I see Julia and am pointing at her." 
