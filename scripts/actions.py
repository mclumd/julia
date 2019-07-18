#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import baxter_interface
from sound_play.libsoundplay import SoundClient


pub = rospy.Publisher('actions', String, queue_size=10)


def raise_arm():
    pub.publish("raising arm")
    return baxter_interface.Limb('right').move_to_joint_positions(
        {'right_e0': -0.013351768775252066, 'right_e1': -0.009032078877376396, 'right_s0': -0.0023561944897503642,
         'right_s1': 0.0019634954081253035, 'right_w0': 0.003141592653000486, 'right_w1': 0.0023561944897503642,
         'right_w2': 0.0019634954081253035})
    pub.publish("done raising arm")


def speak(text="I see julia and am pointing at her"):
    pub.publish("talking")
    voice = 'voice_kal_diphone'
    soundhandle = SoundClient()
    soundhandle.say(text, voice, blocking=True)
    soundhandle.say("", voice, blocking=True)
    pub.publish("done talking")