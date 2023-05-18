#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import copy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from copy import deepcopy

 
class MoveAttachedObjectDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_attached_object_demo')
        
        # 初始化场景对象，用来监听外部环境的变化
        scene = PlanningSceneInterface()
        rospy.sleep(0.5)

   
        # 设置table和tool的三维尺寸
        table_size = [2.4, 1.5, 0.95]
        dropbox_size = [0.6, 0.3, 0.2]
        camera_size = [0.15, 0.02, 0.02]
        
 
        # 将table加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.83
        table_pose.pose.position.z = 0.475
        table_pose.pose.orientation.w = 1.0
        scene.add_box('table', table_pose, table_size)  #添加障碍物
        rospy.sleep(0.5)

        # 将droxbox加入场景当中
        dropbox_pose = PoseStamped()
        dropbox_pose.header.frame_id = 'world'
        dropbox_pose.pose.position.x = -0.6
        dropbox_pose.pose.position.y = 0.25
        dropbox_pose.pose.position.z = 1.0
        dropbox_pose.pose.orientation.w = 1.0
        scene.add_box('dropbox', dropbox_pose, dropbox_size)  #添加障碍物
        rospy.sleep(0.5)
        
        # 将camera加入场景当中
        camera_pose = PoseStamped()
        camera_pose.header.frame_id = 'world'
        camera_pose.pose.position.x = 0.0
        camera_pose.pose.position.y = 0.75
        camera_pose.pose.position.z = 2.0
        camera_pose.pose.orientation.w = 1.0
        scene.add_box('camera', camera_pose, camera_size)  #添加障碍物
        rospy.sleep(0.5)  

        #object_avoidance_pub = rospy.Publisher('object_avoidance', PlanningScene, queue_size=10)
        #object_avoidance_pub.publish(scene)
        #rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    MoveAttachedObjectDemo()
