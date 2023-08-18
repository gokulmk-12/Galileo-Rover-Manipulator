#! /usr/bin/env python3

import rospy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs

class Joy:
    def __init__(self):
        self.outbuff = [0,0,0,0,0,0]
        self.pub=rospy.Publisher('elbow_joint_cmd',std_msgs.Int32MultiArray,queue_size=10)
        self.pub1=rospy.Publisher('joy_teleop_intr',std_msgs.Int32MultiArray,queue_size=10)
        rospy.init_node('ik_joy')
        rospy.Subscriber('joy',sensor_msgs.Joy,self.joy_cb)

    def joy_cb(self,msg):
        outbuff = [0, 0, 0, 0, 0, 0]
        
        outbuff = [ int (msg.axes[i] * 0xFF) for i in range(4) ]
        outbuff += [ (msg.buttons[i] - msg.buttons[i+2]) * 0xFF for i in range(4, 6) ]
        
        axes = [ int (msg.axes[i] * 0xFF) for i in range(5) ]
        buttons = [ (msg.buttons[1] - msg.buttons[2])*255]
        buttons.append((msg.buttons[0] - msg.buttons[3])*255)
        
        outbuff[0] = - axes[0]
        outbuff[1] = axes[1]
        outbuff[2] = axes[3]
        outbuff[3] = buttons[1]
        outbuff[4] = buttons[0]
        outbuff[5] = axes[4]
        
        self.outbuff = outbuff
        #print (self.outbuff)
    
    def start(self):
        rate = rospy.Rate (50)
        while not rospy.is_shutdown():
            rate.sleep()
            msg = self.createMsg (self.outbuff)
            self.pub.publish (msg)
            self.pub1.publish(msg)
    
    def createMsg (self, buff):
        msg = std_msgs.Int32MultiArray()
        msg.data = buff[:]
        
        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0
        
        msg.layout.dim = [ std_msgs.MultiArrayDimension() ]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
        msg.layout.dim[0].label = 'write'
        
        return msg

joy=Joy()
joy.start()
