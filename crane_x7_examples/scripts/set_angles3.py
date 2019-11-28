#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import sys
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

rospy.init_node("pose_groupstate_example")
robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm")
arm.set_max_velocity_scaling_factor(0.9)
gripper = moveit_commander.MoveGroupCommander("gripper")

#SRDF
print("vertical")
arm.set_named_target("vertical")
arm.go()


while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
    rospy.sleep(1.0)

"""
   print("input angle (or type 'end' for finishing)")
    print("example1: a 0 60 (60[deg] for arm joint 0)")
    print("example2: g 0 30 (30[deg] for gripper joint 0)"
 """
data = [["a",1,-20],["a",3,-60],["a",5,-30],["a",3,1],["a",5,1]]
        
for i in range(6):
    #data[i][0],data[i][1],data[i][2]
    part=data[i][0]
    joint=int(data[i][1])
    angle = float(data[i][2])/180.0*math.pi
    print("最初の値は",data[i][0],"でパーツ",data[i][1],"で関節",data[i][2],"で角度を表します")

    print(part, joint, angle)
    if part == "a":
        arm_joint_values = arm.get_current_joint_values()
        arm_joint_values[joint] = angle
        arm.set_joint_value_target(arm_joint_values)
        arm.go()		
    elif part == "g":
        gripper_joint_values = gripper.get_current_joint_values()
        gripper_joint_values[joint] = angle
        gripper.set_joint_value_target(gripper_joint_values)
        gripper.go()		

        
    continue

    #line = sys.stdin.readline()

# 移動後の手先ポーズを表示
arm_goal_pose = arm.get_current_pose().pose
print("Arm goal pose:")
print(arm_goal_pose)
print("done")

