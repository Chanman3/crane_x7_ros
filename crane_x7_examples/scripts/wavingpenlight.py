#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import sys
import moveit_commander
import geometry_msgs.msg
import rosnode

import actionlib
import math
import random
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.msg import ModelStates
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion

gazebo_model_states = ModelStates()

def callback(msg):
    global gazebo_model_states
    gazebo_model_states = msg


def yaw_of(object_orientation):
    # クォータニオンをオイラー角に変換しyaw角度を返す
    euler = euler_from_quaternion(
        (object_orientation.x, object_orientation.y,
        object_orientation.z, object_orientation.w))

    return euler[2]

rospy.init_node("pose_groupstate_example")
robot = moveit_commander.RobotCommander() 
arm = moveit_commander.MoveGroupCommander("arm")   
arm.set_max_velocity_scaling_factor(0.9)
gripper = moveit_commander.MoveGroupCommander("gripper")

#ハンドを閉じる
gripper.set_joint_value_target([0.1,0.1])
gripper.go()


#arm.set_named_target("search")
#arm.go()


# SRDFに定義されている"vertical"の姿勢にする
arm.set_named_target("vertical")
arm.go()

while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
    rospy.sleep(1.0)

data = [ ["a",0,90],["a",1,40],["a",5,30] ,["a",1,-40],["a",5,1],["a",1,1],["a",1,40],["a",5,30],["a",1,-40],["a",5,1],["a",1,1]]
        
for i in range(11):
    part=data[i][0]
    joint=int(data[i][1])
    angle = float(data[i][2])/180.0*math.pi

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

# 移動後の手先ポーズを表示
arm_goal_pose = arm.get_current_pose().pose
print("Arm goal pose:")
print(arm_goal_pose)
print("done")

def main():
    global gazebo_model_states

    OBJECT_NAME = "wood_cube_5cm"   # 掴むオブジェクトの名前
    GRIPPER_OPEN = 1.2              # 掴む時のハンド開閉角度
    GRIPPER_CLOSE = 0.1            # 設置時のハンド開閉角度
    APPROACH_Z = 0.15               # 接近時のハンドの高さ
    LEAVE_Z = 0.20                  # 離れる時のハンドの高さ
    PICK_Z = 0.12                   # 掴む時のハンドの高さ
    PLACE_POSITIONS = [             # オブジェクトの設置位置 (ランダムに設置する)
            Point(0.4, 0.0, 0.0),
            Point(0.0, 0.3, 0.0),
            Point(0.0, -0.3, 0.0),
            Point(0.2, 0.2, 0.0),
            Point(0.2, -0.2, 0.0)]

    sub_model_states = rospy.Subscriber("gazebo/model_states", ModelStates, callback, queue_size=1)

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.4)
    gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()

    # 設置位置に移動する
    #place_position = random.choice(PLACE_POSITIONS) # 設置位置をランダムに選択する
    
    target_pose = Pose()
    target_pose.position.x = 0.4
    target_pose.position.y = 0.0
    target_pose.position.z = 0.1
    
    q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)
    arm.go()
    rospy.sleep(1.0)

    #設置する
    """target_pose.position.z = PICK_Z
    arm.set_pose_target(target_pose) 
    arm.go()
    rospy.sleep(1.0)
    """
    gripper_goal.command.position = GRIPPER_OPEN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))

    # ハンドを上げる
    target_pose.position.z = LEAVE_Z
    arm.set_pose_target(target_pose)
    arm.go()        
    rospy.sleep(1.0)

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()     
    rospy.sleep(1.0)

    print("Done")

    """else:
    print "No objects"
    """

if __name__ == '__main__':
    #rospy.init_node("pick_and_place_in_gazebo_example")
    
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
