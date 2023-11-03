#!/usr/bin/env python3

import sys
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Initialize Moveit Commander and Rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("Galileo_Moveit", anonymous=True)

# Instantiate Robot Commander
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("manipulator")

display_traj = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=10)

joint_goal = group.get_current_joint_values()

joint_goal[0] = math.radians(50)
joint_goal[1] = math.radians(90)
joint_goal[2] = math.radians(-10)
joint_goal[3] = math.radians(10)
joint_goal[4] = math.radians(10)

group.go(joint_goal, wait=True)
group.plan()

rospy.sleep(5)



