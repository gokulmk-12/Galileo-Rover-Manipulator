#!/usr/bin/env python3

import rospy
import numpy as np
import PyKDL as kdl
from geometry_msgs.msg import Pose
import kdl_parser_py.urdf as kdl_parser
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from math import *
import std_msgs.msg as std_msgs
from std_msgs.msg import Float32MultiArray


def joint_list_to_kdl(q):
     if q is None:
         return None
     if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
     q_kdl = kdl.JntArray(len(q))
     for i, q_i in enumerate(q):
         q_kdl[i] = q_i
     return q_kdl

#global variables
f = 0.025
joint_values = []
joint_names = ["Rev1", "Rev2", "Rev3", "Rev4"] 
(status, tree) = kdl_parser.treeFromFile("/home/gokul/ik_ws/src/ik_pkg/config/two_link_arm.urdf")
chain = tree.getChain("base_link", "tip_1")
num_joints = chain.getNrOfJoints()
fk_pos_solver = kdl.ChainFkSolverPos_recursive(chain)
ik_vel_solver = kdl.ChainIkSolverVel_pinv(chain)
ee_pose = kdl.Frame()
target_pose = ee_pose.p
theta_min = joint_list_to_kdl([-1, -2, -2, -3.14])
theta_max = joint_list_to_kdl([2, 2, 2, 3.14])
theta_out = kdl.JntArray(num_joints)
ik_pos_solver = kdl.ChainIkSolverPos_NR_JL(chain, theta_min, theta_max, fk_pos_solver, ik_vel_solver)


def endeff_pose(data):
        global ee_pose, joint_values
        joint_values = data.position
        fk_pos_solver.JntToCart(joint_list_to_kdl(joint_values), ee_pose)  
        
def goal_set(data,args):
        global target_pose
        target_pose = ee_pose.p
        target_pose[0] += f*data.position.x
        target_pose[1] += f*data.position.y
        target_pose[2] += f*data.position.z
        desired_pose = ee_pose
        desired_pose.p[0], desired_pose.p[1], desired_pose.p[2] = target_pose[0], target_pose[1], target_pose[2] 
        ik_pos_solver.CartToJnt(joint_list_to_kdl(joint_values),desired_pose, theta_out)
        js = JointState()
        for i in range(4):
            js.name.append(joint_names[i])
            js.position.append(theta_out[i])
            js.velocity.append(0)
            js.effort.append(0)
        args[0].publish(js)

        msg=std_msgs.Float32MultiArray()
        msg.data=js.position

        msg.layout=std_msgs.MultiArrayLayout()
        msg.layout.data_offset=0

        msg.layout.dim=[std_msgs.MultiArrayDimension()]
        msg.layout.dim[0].size=msg.layout.dim[0].stride=len(msg.data)
        msg.layout.dim[0].label="joint_angles"
        pub1.publish(msg)

        
if __name__ == "__main__":
    rospy.init_node("iksolver_node", anonymous=True)
    rviz_pub = rospy.Publisher("/rviz_input", JointState, queue_size=10)
    pub1=rospy.Publisher('/ik_joint_angles',Float32MultiArray,queue_size=10)
    rospy.Subscriber("/joint_states", JointState, endeff_pose)
    rospy.Subscriber("/endeff_input", Pose, goal_set,callback_args=[rviz_pub])
    rospy.spin()    



