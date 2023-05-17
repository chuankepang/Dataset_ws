#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import _thread, copy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from copy import deepcopy


 
class MoveItIkDemo:
    def __init__(self):
 
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')

	    # 初始化场景对象，用来监听外部环境的变化
        scene = PlanningSceneInterface()
        rospy.sleep(1)
                
        # 初始化需要使用move group控制的机械臂中的arm group

        gripper = moveit_commander.MoveGroupCommander('gripper') 

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        gripper.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        gripper.allow_replanning(True)


        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        gripper.set_goal_position_tolerance(5)
        gripper.set_goal_orientation_tolerance(5)
        
        

        gripper.set_named_target("close")
        gripper.go()

        rospy.sleep(2)

        gripper.set_named_target("open")
        gripper.go()

        rospy.sleep(2)

	
       
 
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    MoveItIkDemo()

