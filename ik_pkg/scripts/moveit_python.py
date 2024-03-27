#!/usr/bin/env python3

import tf
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

# display_traj = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=10)

joint_goal = group.get_current_joint_values()
br = tf.TransformBroadcaster()

while not rospy.is_shutdown() :

    br.sendTransform([0.6, 0.0, 0.12],[0.0, 0.0, 0.0, 1.0],rospy.Time.now(), "Obj_1", "base_link")
    rospy.sleep(2)

    joint_goal[0] = math.radians(1)
    joint_goal[1] = math.radians(44)
    joint_goal[2] = math.radians(-59)
    joint_goal[3] = math.radians(-48)
    joint_goal[4] = math.radians(-3)

    group.go(joint_goal, wait=True)
    # group.execute(joint_goal)
    rospy.sleep(2)

    br.sendTransform([0.7, 0.43, 0.15],[-0.1135, 0.11883, 0.27453, 0.94743],rospy.Time.now(), "Obj_2", "base_link")

    joint_goal[0] = math.radians(31)
    joint_goal[1] = math.radians(65)
    joint_goal[2] = math.radians(-22)
    joint_goal[3] = math.radians(-13)
    joint_goal[4] = math.radians(9)

    group.go(joint_goal, wait=True)
    # group.plan()
    rospy.sleep(3)

    break

    # joint_goal[0] = math.radians(40)
    # joint_goal[1] = math.radians(90)
    # joint_goal[2] = math.radians(-10)
    # joint_goal[3] = math.radians(10)
    # joint_goal[4] = math.radians(10)

    # group.go(joint_goal, wait=True)
    # group.plan()
    # rospy.sleep(5)



