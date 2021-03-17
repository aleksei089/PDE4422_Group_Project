#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group1 = moveit_commander.MoveGroupCommander("arm")
group2 = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

group1_variable_values = group1.get_current_joint_values()
group2_variable_values = group2.get_current_joint_values()

group1_variable_values[0] = 0
group1_variable_values[1] = 0
group1_variable_values[2] = 0
group1_variable_values[3] = 0
group2_variable_values[0] = 0.019
group2_variable_values[1] = 0.019
group1.set_joint_value_target(group1_variable_values)
group2.set_joint_value_target(group2_variable_values)

plan2 = group1.plan()
group1.go(wait=True)
plan3 = group2.plan()
group2.go(wait=True)

group1_variable_values[0] = 0.000
group1_variable_values[1] = 0.680
group1_variable_values[2] = 0.000
group1_variable_values[3] = -0.620
group2_variable_values[0] = 0.002
group2_variable_values[1] = 0.002
group1.set_joint_value_target(group1_variable_values)
group2.set_joint_value_target(group2_variable_values)

plan2 = group1.plan()
group1.go(wait=True)
plan3 = group2.plan()
group2.go(wait=True)

group1_variable_values[0] = 0
group1_variable_values[1] = 0
group1_variable_values[2] = 0
group1_variable_values[3] = 0.001
group2_variable_values[0] = 0.002
group2_variable_values[1] = 0.002
group1.set_joint_value_target(group1_variable_values)
group2.set_joint_value_target(group2_variable_values)

plan2 = group1.plan()
group1.go(wait=True)
plan3 = group2.plan()
group2.go(wait=True)

moveit_commander.roscpp_shutdown()
