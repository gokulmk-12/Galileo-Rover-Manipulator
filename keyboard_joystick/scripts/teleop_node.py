#!/usr/bin/env python3

from __future__ import division
import rospy
import os
import std_msgs.msg as std_msgs
from geometry_msgs.msg import Pose

outbuff=[]

def joy_clb(array):
    global outbuff
    outbuff=array.data
    #print(outbuff)
    return outbuff

if __name__ == "__main__":
    rospy.init_node('joy_teleop_node', anonymous=True)
    pub = rospy.Publisher('/endeff_input', Pose, queue_size=10)
    rospy.Subscriber('joy_teleop_intr',std_msgs.Int32MultiArray,joy_clb)

    rate=rospy.Rate(50)

    while not rospy.is_shutdown():

        rate.sleep()
        msg=Pose()

        msg.position.x=0
        msg.position.y = 0
        msg.position.z = 0

        if len(outbuff) == 6:
            if (outbuff[0] < 0):
                msg.position.x=-1
                print("x left")
            if (outbuff[0] > 0):
                msg.position.x=1
                print("x right")
            if (outbuff[1] > 0):
                msg.position.y=1
                print("y front")
            if (outbuff[1] < 0):
                msg.position.y=-1
                print("y back")
            if (outbuff[5] > 0):
                msg.position.z=1
                print("z up")
            if (outbuff[5] < 0):
                msg.position.z=-1
                print("z down")
        
        pub.publish(msg)


    







