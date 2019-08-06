#!/usr/bin/env python

"""
'Main' script for julia demo

Mostly just initializes stuff
Still handles some voice recognition stuff while ALMA can't handle strings

Voice Recognition (pocketsphinx) stuff:
pocketsphinx uses a language model and dictionary based on a set of words it needs to be able to recognize
These are all available in the config folder, and new ones can be generated using their online tool
Note that these must be referenced in the launch file
"""

import rospy
import baxter_interface
from std_msgs.msg import String


rospy.init_node('juliademo')
rs = baxter_interface.RobotEnable()
alma_comm = rospy.Publisher('alma_in', String, queue_size=10)


def main():
    rospy.on_shutdown(clean_shutdown)
    rs.enable()

    rospy.Subscriber('/recognizer/output', String, point)

    rospy.spin()


def point(data):
    # Prints out whatever speech is recognized
    rospy.loginfo(data.data)
    # Not very general, needed because ALMA can't handle arbitrary strings
    # In the future, ALMA would process speech stuff the same as vision stuff (see detect.py)
    if 'julia' in data.data:
        alma_comm.publish("obs hearing(julia).\n")


def clean_shutdown():
    rs.disable()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
