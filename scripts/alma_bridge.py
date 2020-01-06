#!/usr/bin/env python

"""

ROS Node to run ALMA
Passes commands from /alma_in rostopic to ALMA command line
Evaluates ROS commands that are the only predicate in the line

If you get an error about command not recognized and when you quit terminal it prints a bunch of empty lines,
You probably forgot a command at the beginning (add, obs), a . at the end, or a \n at the end
Also, if ALMA doesn't idle correctly, this will keep running

"""

import subprocess
import rospy
import time
from std_msgs.msg import String
from actions import *

alma = subprocess.Popen(["./alma.x", "demo/julia.pl"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, shell=False, cwd="./src/julia/alma/")
looping = False




# Basic logic:
# ALMA will keep stepping until idling
# Then it will wait until something is added
# But adding can still happen at any time, so it doesn't have to be idling to add something
# "Add" here meaning feed some input to the ALMA command line
def main():
    rospy.init_node('almabridge')
    rospy.Subscriber("alma_in", String, add)
    init_lowered_angles()
    loop()
    rospy.spin()


# Match up parentheses so we can figure out if a line is just a single ROS command
def find_parens(s):
    toret = {}
    pstack = []

    for i, c in enumerate(s):
        if c == '(':
            pstack.append(i)
        elif c == ')':
            toret[pstack.pop()] = i

    return toret


def loop():
    global looping
    looping = True
    idle = False
    output = alma.stdout.readline()
    # Ugh this is bad
    # step and print end in a newline, so they're fine
    # However, these others print just one line saying "____ added" or something similar
    # So we need to check for those
    # If a new ALMA command gets added that acts like these, then:
    # it would need to be added,
    # ALMA would need to change its output,
    # or this would need to be handled better

    # Flush output from a prior command
    while output != "\n" and not "added" in output and not "removed" in output and not "observed" in output:
        print(output)
        output = alma.stdout.readline()

    while not idle:

        # print, read through for any ROS commands, then step
        alma.stdin.write('print\n')
        output = alma.stdout.readline()
        roscommands = []
        while output != "\n" and not "added" in output and not "observed" in output:
            # while not "alma:" in output:
            print(output)
            if len(output.split(": ros(")) > 1:
                output = output.split(": ros")[1].split("(parents:")[0].strip()
                # This will only consider a ros command if the line is a single command
                # e.g. ros(speak())
                # Note that the commands need parens like a function would
                if output[:find_parens(output)[0] + 1] == output:
                    # We need to check if these are for the current time step
                    # But now() is always the last printed, so hold onto these until we know what time it is
                    roscommands.append(output[1:-1])

            # Find now()
            if len(output.split(": now(")) > 1:
                t = output.split(": now(")[1].split(")")[0]
                for command in roscommands:
                    # Only evaluate a ROS command if it's for the current time (actually time-1)
                    if command.split("),")[1].strip() == str(int(t) - 1):
                        print("ROS COMMAND: " + command.split(",")[0])
                        eval(command.split(")")[0] + ")")

            output = alma.stdout.readline()

        alma.stdin.write('step\n')
        alma.stdin.write('\n')
        output = alma.stdout.readline()
        if "Idling..." in output:
            idle = True

    looping = False


def add(data):
    alma.stdin.write(data.data)
    # If it was idling, start up again because of new information
    if not looping:
        loop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
