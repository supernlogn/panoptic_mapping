#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped, Pose
from visualization_msgs.msg import Marker

import numpy as np


class PathVisualizer(object):
    def __init__(self):
        """  Initialize ros node and read params """
        # Params
        self.red = rospy.get_param('~r', 1)  # color 0-1
        self.green = rospy.get_param('~g', 0)
        self.blue = rospy.get_param('~b', 0)
        self.length = rospy.get_param('~length', 0.3)  # m
        self.width = rospy.get_param('~width', 0.03)  # m
        self.distance = rospy.get_param('~distance', 0.05)  # m
        self.use_arrow = rospy.get_param('~use_arrow', True)  # m
        # ROS
        self.sub = rospy.Subscriber("~pose_in", TransformStamped,
                                    self.transform_cb)
        self.pub = rospy.Publisher("~marker_out", Marker, queue_size=10)
        # variables
        self.previous_pose = None
        self.counter = 0

    def transform_cb(self, transform_stamped):
        # Init
        if self.previous_pose is None:
            self.previous_pose = self.transformToPose(
                transform_stamped.transform)
            return

        # Only plot every 'distance' meters

        pose = self.transformToPose(transform_stamped.transform)
        dist = np.linalg.norm(
            np.array([
                self.previous_pose.position.x, self.previous_pose.position.y,
                self.previous_pose.position.z
            ]) - np.array([pose.position.x, pose.position.y, pose.position.z]))
        if dist < self.distance:
            return
        self.previous_pose = pose

        # Plot
        msg = Marker()
        msg.header = transform_stamped.header

        msg.pose = pose
        msg.color.r = self.red
        msg.color.g = self.green
        msg.color.b = self.blue
        msg.color.a = 1
        msg.ns = "path"
        if self.use_arrow:
            msg.type = 0
            msg.scale.x = self.length
            msg.scale.y = self.width
            msg.scale.z = self.width * 2
        else:
            # Sphere
            msg.type = 2
            msg.scale.x = self.width
            msg.scale.y = self.width
            msg.scale.z = self.width
        msg.id = self.counter
        self.counter = self.counter + 1
        self.pub.publish(msg)

    def transformToPose(self, transform):
        p = Pose()
        p.position.x = transform.translation.x
        p.position.y = transform.translation.y
        p.position.z = transform.translation.z
        p.orientation.x = transform.rotation.x
        p.orientation.y = transform.rotation.y
        p.orientation.z = transform.rotation.z
        p.orientation.w = transform.rotation.w
        return p


if __name__ == '__main__':
    rospy.init_node('path_visualizer', anonymous=True)
    pv = PathVisualizer()
    rospy.spin()
