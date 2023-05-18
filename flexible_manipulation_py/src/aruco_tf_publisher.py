#!/usr/bin/env python3
# #-*- coding: UTF-8 -*- 

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
import tf

# 初始化ROS节点
rospy.init_node("publisher")
# 创建话题发布器，话题名称为"msg"，类型为String
pub = rospy.Publisher("aruco_tf", PoseStamped, queue_size=10)
# 设置循环频率，单位: Hz
loop_rate = rospy.Rate(10)
cnt = 0
while not rospy.is_shutdown():
    arucotf = PoseStamped()
    tfl = tf.TransformListener()
    # if tfl.canTransform("/world", "/camera_marker", rospy.Time().now()):
    tfl.waitForTransform("/world", "/camera_marker", rospy.Time(), rospy.Duration(1.0))
    (trans,rot) = tfl.lookupTransform("/world", "/camera_marker", rospy.Time(0))
    cnt = cnt + 1
    arucotf.pose.position.x = trans[0]
    arucotf.pose.position.y = trans[1]
    arucotf.pose.position.z = trans[2]

    arucotf.pose.orientation.x = 0
    arucotf.pose.orientation.y = 0
    arucotf.pose.orientation.z = 0
    arucotf.pose.orientation.w = 1
    # 打印发布的消息
    rospy.loginfo(arucotf)
    # 发布消息
    pub.publish(arucotf)
    # 休眠
    loop_rate.sleep()
