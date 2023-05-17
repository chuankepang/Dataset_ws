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
import tf

def tf_listener():

    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)

    # 初始化ROS节点
    rospy.init_node('moveit_ik_demo')

    # 初始化场景对象，用来监听外部环境的变化
    scene = PlanningSceneInterface()
    # rospy.sleep(1)
            
    # 初始化需要使用move group控制的机械臂中的arm group
    ur = moveit_commander.MoveGroupCommander('manipulator')
    gripper = moveit_commander.MoveGroupCommander('gripper') 

    # 获取终端link的名称，这个在setup assistant中设置过了
    end_effector_link = 'yixiuge_ee_link'

                    
    # 设置目标位置所使用的参考坐标系
    reference_frame = 'world'
    ur.set_pose_reference_frame(reference_frame)

            
    # 当运动规划失败后，允许重新规划
    ur.allow_replanning(True)


    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    ur.set_goal_position_tolerance(0.03)
    ur.set_goal_orientation_tolerance(0.03)


    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    gripper.set_goal_position_tolerance(0.03)
    gripper.set_goal_orientation_tolerance(0.03)

    # 设置允许的最大速度和加速度
    ur.set_max_acceleration_scaling_factor(0.5)
    ur.set_max_velocity_scaling_factor(0.5)
    ur.set_max_acceleration_scaling_factor(0.5)
    ur.set_max_velocity_scaling_factor(0.5)

    # 控制机械臂先回到初始化位置
    ur.set_named_target('yixiuge_home')
    ur.go()

    rospy.sleep(1)

    #gripper.set_named_target('open')
    #gripper.go()

    #rospy.sleep(2)


    # rospy.init_node('py_coordinate_transformation')
    tfl = tf.TransformListener()
    # rate = rospy.Rate(125.0)
    # while not rospy.is_shutdown():
    try:
        (trans,rot) = tfl.lookupTransform('world', 'camera_marker', rospy.Time(0))
        print(trans)
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose1 = PoseStamped()


        #参考坐标系，前面设置了
        target_pose1.header.frame_id = reference_frame
        target_pose1.header.stamp = rospy.Time.now() #时间戳？

        #末端位置2，右手预备位置1   
        target_pose1.pose.position.x = trans[0]
        target_pose1.pose.position.y = trans[1] - 0.1
        target_pose1.pose.position.z = trans[2] + 0.6
        #末端姿态，四元数
        target_pose1.pose.orientation.x = 0
        target_pose1.pose.orientation.y = 0
        target_pose1.pose.orientation.z = 0
        target_pose1.pose.orientation.w = 1

        # 设置机器臂当前的状态作为运动初始状态

        ur.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿

        ur.set_pose_target(target_pose1, end_effector_link)
        ur.go()

        # 规划运动路径，返回虚影的效果

        # traj = ur.plan()
        #plan_success, traj1, planning_time, error_code = ur.plan()

        # 按照规划的运动路径控制机械臂运动

        #ur.execute(traj1)
        # socket_opengripper()
        # gripper_open_publisher()

        #rospy.sleep(2)  #执行完成后休息1s



        # 关闭并退出moveit
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)
        # rate.sleep()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #continue
        #return (trans,rot)

if __name__ == "__main__":
    tf_listener()
