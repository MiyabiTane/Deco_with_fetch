#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import Point, Pose, Quaternion, PoseArray

import numpy as np

class PolygonToVector(object):
    def __init__(self):
        self.pub = rospy.Publisher("~output", PoseArray, queue_size=1)
        rospy.Subscriber("~input", PolygonArray, self.plane_vec_cb)
        
    def polygon_to_vec(self, points_msg):
        # 同一平面上の点なので任意の3点で良い
        p1_msg = points_msg[0]
        p2_msg = points_msg[int(len(points_msg)/2)]
        p3_msg = points_msg[len(points_msg)-1]
        p1 = np.array([p1_msg.x, p1_msg.y, p1_msg.z])
        p2 = np.array([p2_msg.x, p2_msg.y, p2_msg.z])
        p3 = np.array([p3_msg.x, p3_msg.y, p3_msg.z])
        vec_1 = p1 - p2
        vec_2 = p3 - p2
        norm_vec = np.cross(vec_1, vec_2)
        vec_size = np.linalg.norm(norm_vec)
        return norm_vec / vec_size, p2
    
    def plane_vec_cb(self, msg):
        pose_array_msg = PoseArray()
        pose_array_msg.poses = []
        for polygon in msg.polygons:
            norm_vec, center_pos = self.polygon_to_vec(polygon.polygon.points)
            point_msg = Point()
            quaternion_msg = Quaternion()
            pose_msg = Pose()
            point_msg.x = center_pos[0]
            point_msg.y = center_pos[1]
            point_msg.z = center_pos[2]
            quaternion_msg.x = norm_vec[0]
            quaternion_msg.y = norm_vec[1]
            quaternion_msg.z = norm_vec[2]
            pose_msg.position = point_msg
            pose_msg.orientation = quaternion_msg
            pose_array_msg.poses.append(pose_msg)
        self.pub.publish(pose_array_msg)

if __name__ == '__main__':
    rospy.init_node("polygon_to_vector")
    point_to_vector = PolygonToVector()
    rospy.spin()
