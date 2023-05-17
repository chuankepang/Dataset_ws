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
    rospy.init_node('py_coordinate_transformation')
    tfl = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = tfl.lookupTransform('world', 'camera_marker', rospy.Time(0))
            print(trans, rot)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return (trans,rot)
    rate.sleep()


if __name__ == '__main__':
    tf_listener()