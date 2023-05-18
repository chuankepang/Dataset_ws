#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
import tf
import threading
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose




# 初始化move_group的API
moveit_commander.roscpp_initialize(sys.argv)               
# 初始化需要使用move group控制的机械臂中的arm group
arm = moveit_commander.MoveGroupCommander('manipulator')
# 初始化需要使用move group控制的机械臂中的gripper group
gripper = moveit_commander.MoveGroupCommander('gripper')       
# 获取终端link的名称
end_effector_link = 'yixiuge_ee_link'
# 设置目标位置所使用的参考坐标系
reference_frame = 'world'
arm.set_pose_reference_frame(reference_frame)              
# 当运动规划失败后，允许重新规划
arm.allow_replanning(True)      
# 设置位置(单位：米)和姿态（单位：弧度）的允许误差
arm.set_goal_position_tolerance(0.01)
arm.set_goal_orientation_tolerance(0.05)
gripper.set_goal_joint_tolerance(0.001)        
# 控制机械臂先回到初始化位置
arm.set_named_target('yixiuge_home')
arm.go()
# 设置机器臂当前的状态作为运动初始状态
arm.set_start_state_to_current_state()

a = 1 
def Listener():
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("aruco_tf", PoseStamped, ar_pose)
        rospy.spin()
def ar_pose(data):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        
        ox = data.pose.orientation.x
        oy = data.pose.orientation.y
        oz = data.pose.orientation.z
        ow = data.pose.orientation.w
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x =  x
        target_pose.pose.position.y = y-0.1
        target_pose.pose.position.z = z+0.6
        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 1
        print(target_pose)
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()
        global a
        a+=1
        print(" count ",a) 
     # 关闭并退出moveit
        #moveit_commander.roscpp_shutdown()
        #moveit_commander.os._exit(0)
        arm.clear_pose_targets()
        print("清除") 
if __name__ == "__main__":
    Listener()