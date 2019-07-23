#!/usr/bin/env python

#######3run roscore, then in new terminal baxter sh run soundplay node, then in new terminal bbaxter sh run this
####Run  roscore in a regular terminal
##open second, run baxter sh, then enable robot, then run soundplay node
####

####install ros
#####install sound play
###install pocketsphinx
##install gstreamer for pocketsphinx


# import rospy
from actions import *
import baxter_interface
from std_msgs.msg import String
from sound_play.msg import SoundRequest

from sound_play.libsoundplay import SoundClient

voice = 'voice_kal_diphone'
soundhandle = SoundClient()
rospy.init_node('juliademo')
rs = baxter_interface.RobotEnable()
current_actions = []
alma_comm = rospy.Publisher('alma_in', String, queue_size=10)


def alma(data):
    rospy.loginfo(data.data)
    print(data.data)
    eval(data.data)


def main():
    rospy.on_shutdown(clean_shutdown)
    rs.enable()
    # print(baxter_interface.Limb('right').joint_names())

    # baxter_interface.Limb('right').set_joint_positions({'right_e1': 1})
    # baxter_interface.Limb('right').move_to_joint_positions({'right_s0': -0.4724660821289063, 'right_s1': -0.23623304106445314, 'right_w0': 0.02032524541625977,
    # 'right_w1': 0.25885925765991213, 'right_w2': 0.027611654150390626, 'right_e0': 0.13230584280395508,
    # 'right_e1': 1.6739565328674317})
    """pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    """
    # soundhandle.say("My name is bob", voice)
    global listener

    listener = rospy.Subscriber('/recognizer/output', String, point)
    rospy.Subscriber("actions", String, update)
    rospy.Subscriber("alma", String, alma)

    rospy.spin()


def update(data):
    rospy.loginfo("****UPDATE****")
    global listener
    rospy.loginfo(current_actions)
    if "done" in data.data:
        current_actions.remove(data.data[5:])
    else:
        current_actions.append(data.data)

    # smart bit
    if "talking" in current_actions:
        rospy.loginfo("UNREGISTER")
        listener.unregister()
        listener = None
    elif not listener:
        rospy.loginfo("REGISTER")
        listener = rospy.Subscriber('/recognizer/output', String, point)

    rospy.loginfo(current_actions)
    rospy.loginfo("****END UPDATE****")


def point(data):
    rospy.loginfo("****POINT****")
    rospy.loginfo(data.data)
    rospy.loginfo(current_actions)
    if 'julia' in data.data:

        alma_comm.publish("hearing(julia)")
        if not "talking" in current_actions:
            alma_comm.publish("not(talking)")

        # raise_arm()
        # speak("I see julia and am pointing at her")

        # soundhandle.say("I see julia and am pointing at her", voice)

        # baxter_interface.Limb('right').move_to_joint_positions({'right_e0': -0.013351768775252066, 'right_e1': -0.009032078877376396, 'right_s0': -0.0023561944897503642, 'right_s1': 0.0019634954081253035, 'right_w0': 0.003141592653000486, 'right_w1': 0.0023561944897503642, 'right_w2': 0.0019634954081253035})

        """rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            # hello_str = "hello world %s" % rospy.get_time()
            # rospy.loginfo(hello_str)
            # pub.publish(hello_str)
            print(baxter_interface.Limb('right').joint_angle('right_e1'))
            baxter_interface.Limb('right').set_joint_positions(
                {'right_s0': -0.4724660821289063, 'right_s1': -0.23623304106445314, 'right_w0': 0.02032524541625977,
                 'right_w1': 0.25885925765991213, 'right_w2': 0.027611654150390626, 'right_e0': 0.13230584280395508,
                 'right_e1': 1.6739565328674317})
            ####Figure out when to stop then loop it

            rate.sleep()"""
    rospy.loginfo("****END POINT****")


def clean_shutdown():
    rs.disable()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
