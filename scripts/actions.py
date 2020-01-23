#!/usr/bin/env python

"""
Library of functions for Baxters

Each action will communicate with alma when it starts and stops


Right now it signals a finished action with not(action), which triggers a contradiction and distrusts both
This is kind of a hack until ALMA has something to signal when an action is done, to put it in history
Right now distrust is being used as a history container
"""

import rospy
from std_msgs.msg import String
import baxter_interface
import os
#from sound_play.libsoundplay import SoundClient

pub = rospy.Publisher('alma_in', String, queue_size=10)

lowered_angles = None

def init_lowered_angles():
    global lowered_angles
    lowered_angles = baxter_interface.Limb('right').joint_angles()
    print(lowered_angles)

def raise_arm(x, y):
    pub.publish("add raising_arm.\n")
    baxter_interface.Limb('right').move_to_joint_positions(
        {'right_e0': -0.013351768775252066, 'right_e1': -0.009032078877376396,
         'right_s0': 1.5865196298704891 - x * 1.41700024325
                     / 1920,
         'right_s1': -0.608553499609337 + y * 0.93275256544 / 1080, 'right_w0': 0.003141592653000486,
         'right_w1': 0.0023561944897503642,
         'right_w2': 0.0019634954081253035})
    pub.publish("add not(raising_arm).\n")

def lower_arm():
    global lowered_angles
    pub.publish("add lowering_arm.\n")
    baxter_interface.Limb('right').move_to_joint_positions(lowered_angles)
    pub.publish("add not(lowering_arm).\n")


def speak(text="I see julia and am pointing at her"):
    pub.publish("add talking.\n")
#    voice = 'voice_kal_diphone'
#    soundhandle = SoundClient()
#    soundhandle.say(text, voice, blocking=True)
#    soundhandle.say("", voice, blocking=True)
    os.system("mpg321 julia.mp3")
    lower_arm()
    pub.publish("add not(talking).\n")
