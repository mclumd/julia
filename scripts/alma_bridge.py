#!/usr/bin/env python
import re
import subprocess
import rospy
from std_msgs.msg import String
import sys

# subprocess.call(["ls", "-l"])
# subprocess.call("./../alma.alma.x")

alma = subprocess.Popen(["./alma.x", "demo/julia.pl"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, shell=False, cwd="/home/mcl/ros_ws/src/julia/alma/")
pub = rospy.Publisher('alma', String, queue_size=10)
looping = False


def main():
    # alma.stdin.write('halt\n')
    # print(alma.communicate(input=b'print')[0])

    """alma.stdin.write('print\n')
    while alma.stdout.readline()!="alma: Idling...\n":
        alma.stdin.write('step\n')
    print("out")"""
    rospy.init_node('almabridge')
    rospy.Subscriber("alma_in", String, add)
    loop()
    rospy.spin()


def flush():
    output = alma.stdout.readline()
    while output != "\n" and not "added" in output:
        # while not "alma:" in output:
        print(output)
        output = alma.stdout.readline()
    # print("exited on:"+output+":e")
    """
    usin=""
    while not "q" in usin:
        usin=raw_input("go now")
        print(alma.stdout.readline())"""


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
    """
    usin=""
    while not "q" in usin:
        print(alma.stdout.readline())
        usin=raw_input("hey")

        """
    global looping
    looping = True
    idle = False
    output = alma.stdout.readline()
    while output != "\n" and not "added" in output and not "removed" in output:
        # while not "alma:" in output:
        print(output)
        output = alma.stdout.readline()
    # print("exited on:" + output + ":e")
    while not idle:
        # print("loop")
        # print(idle)

        # print("printing:")
        alma.stdin.write('print\n')
        output = alma.stdout.readline()
        while output != "\n" and not "added" in output:
            # while not "alma:" in output:
            print(output)
            if len(output.split(": ros(")) > 1:
                output = output.split(": ros")[1].split("(parents:")[0].strip()
                # print("ros::"+output)
                # print(find_parens(output))
                # print(output[:find_parens(output)[0]])
                if output[:find_parens(output)[0] + 1] == output:
                    print("ROS COMMAND: " + output[1:-1])
                    pub.publish(output[1:-1])
                    #alma.stdin.write('del ros(' + output[1:-1] + ').\n')

            output = alma.stdout.readline()
        # print("exited on:" + output + ":e")
        # print("half")
        # idle=True

        alma.stdin.write('step\n')
        alma.stdin.write('\n')
        output = alma.stdout.readline()
        if "Idling..." in output:
            # print("setting idle")
            idle = True
        """while output != "\n" and not idle:
            print("a"+output+"ab")
            if "Idling..." in output:
                print("setting idle")
                idle=True
            else:
                output = alma.stdout.readline()"""

        """
        if len(output.split(": "))>1 and output.split(": ")[1][:3]=="ros(":
            pub.publish(output.split(": ")[1])
        
        if not output:
            print('[No more data]')
            break
            """

        # if output !="\n":
        # print("here")
    looping = False
    # print("done looping")


def add(data):
    # print("a"+data.data+"a")
    # print(looping)
    alma.stdin.write("add " + data.data + ".\n")
    if not looping:
        loop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

###write something to keep printing alma, filter out the ros stuff and publish it
####dont inherit stuff with ros symbol

#####add hearing
