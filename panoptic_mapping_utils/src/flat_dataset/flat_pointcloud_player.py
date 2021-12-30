#!/usr/bin/env python

import os
import csv
from struct import pack, unpack

from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import rospy
import tf
from PIL import Image as PilImage

import numpy as np
import cv2


class FlatDataPlayer(object):
    def __init__(self):
        """ Initialize ros node and read params """
        # params
        self.data_path = rospy.get_param(
            '~data_path', '/home/ioannis/datasets/flat_dataset/run1')
        self.global_frame_name = rospy.get_param('~global_frame_name', 'world')
        self.sensor_frame_name = rospy.get_param('~sensor_frame_name',
                                                 "depth_cam")
        self.play_rate = rospy.get_param('~play_rate', 1.0)
        self.wait = rospy.get_param('~wait', False)
        self.max_frames = rospy.get_param('~max_frames', 1e9)
        self.refresh_rate = rospy.get_param('~refresh_rate', 1)  # Hz
        # ROS
        self.pointCloud_pub = rospy.Publisher("~pointcloud",
                                              PointCloud2,
                                              queue_size=100)
        self.pose_pub = rospy.Publisher("~pose",
                                        TransformStamped,
                                        queue_size=100)
        # camera
        self.cameraInfo = rospy.get_param("~camera")
        self.maximum_distance = self.cameraInfo['max_range']
        self.minimum_distance = self.cameraInfo['min_range']
        self.use_detectron = False
        self.flatten_distance = self.maximum_distance
        self.tf_broadcaster = tf.TransformBroadcaster()
        # setup
        self.cv_bridge = CvBridge()
        stamps_file = os.path.join(self.data_path, 'timestamps.csv')
        self.times = []
        self.ids = []
        self.current_index = 0  # Used to iterate through
        if not os.path.isfile(stamps_file):
            rospy.logfatal("No timestamp file '%s' found." % stamps_file)
        with open(stamps_file, 'r') as read_obj:
            csv_reader = csv.reader(read_obj)
            for row in csv_reader:
                if row[0] == "ImageID":
                    continue
                self.ids.append(str(row[0]))
                self.times.append(float(row[1]) / 1e9)

        self.ids = [x for _, x in sorted(zip(self.times, self.ids))]
        self.times = sorted(self.times)
        self.times = [(x - self.times[0]) for x in self.times]
        self.start_time = None

        if self.wait:
            self.start_srv = rospy.Service('~start', Empty, self.start)
        else:
            self.start(None)

    def start(self, _):
        self.running = True
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.refresh_rate),
                                 self.callback)
        return EmptyResponse()

    def callback(self, _):
        # Check we should be publishing.
        if not self.running:
            return

        # Check we're not done.
        if self.current_index >= len(self.times):
            rospy.loginfo("Finished playing the dataset.")
            rospy.signal_shutdown("Finished playing the dataset.")
            return

        # Check the time.
        now = rospy.Time.now()
        if self.start_time is None:
            self.start_time = now
        if self.times[self.current_index] > self.play_rate * (
                now - self.start_time).to_sec():
            return
        now = rospy.Time.from_sec(self.times[self.current_index])
        # Get all data and publish.
        file_id = os.path.join(self.data_path, self.ids[self.current_index])

        # Color
        color_file = file_id + "_color.png"
        depth_file = file_id + "_depth.tiff"
        pose_file = file_id + "_pose.txt"
        pred_file = file_id + "_segmentation.png"
        files = [color_file, depth_file, pose_file, pred_file]

        for f in files:
            if not os.path.isfile(f):
                rospy.logwarn("Could not find file '%s', skipping frame." % f)
                self.current_index += 1
                return

        # Load and publish Color image.
        img_color = np.array(cv2.imread(color_file))
        rgb = self.rgb_to_float(img_color)

        # Load and publish depth image. These are optional.
        img_depth = np.array(PilImage.open(depth_file))
        mask_depth = img_depth.reshape(-1)
        img_depth = np.clip(img_depth, self.minimum_distance,
                            self.flatten_distance)
        (x, y, z) = self.depth_to_3d(img_depth)
        (x, y, z) = (x, y, z)
        # Remove invalid points
        if self.maximum_distance > 0:
            mask = mask_depth <= self.maximum_distance
            mask = np.logical_and(mask, mask_depth >= self.minimum_distance)
            x = x[mask]
            y = y[mask]
            z = z[mask]
            rgb = rgb[mask]
            # img_depth = img_depth[mask]

        # Publish pointcloud
        data = np.transpose(np.vstack((x, y, z, rgb)))
        msg = PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = 'camera'
        msg.width = data.shape[0]
        msg.height = 1
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.float32(data).tostring()
        self.pointCloud_pub.publish(msg)

        # Load and publish transform.
        if os.path.isfile(pose_file):
            pose_data = [float(x) for x in open(pose_file, 'r').read().split()]
            transform = np.eye(4)
            for row in range(4):
                for col in range(4):
                    transform[row, col] = pose_data[row * 4 + col]
            rotation = tf.transformations.quaternion_from_matrix(transform)
            position = transform[:, 3]

        pose_msg = TransformStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.global_frame_name
        pose_msg.transform.translation.x = position[0]
        pose_msg.transform.translation.y = position[1]
        pose_msg.transform.translation.z = position[2]

        pose_msg.transform.rotation.x = rotation[0]
        pose_msg.transform.rotation.y = rotation[1]
        pose_msg.transform.rotation.z = rotation[2]
        pose_msg.transform.rotation.w = rotation[3]
        pose_msg.child_frame_id = "camera"
        self.pose_pub.publish(pose_msg)
        # self.tf_broadcaster.sendTransform(position, rotation, now, "camera",
        #                                   "world")

        self.current_index += 1
        if self.current_index > self.max_frames:
            rospy.signal_shutdown("Played reached max frames (%i)" %
                                  self.max_frames)

    def depth_to_3d(self, img_depth):
        """ Create point cloud from depth image and camera params.
            Returns a single array for x, y and z coords """
        # read camera params and create image mesh
        height = self.cameraInfo['height']
        width = self.cameraInfo['width']
        center_x = width / 2
        center_y = height / 2
        f = self.cameraInfo['fx']
        cols, rows = np.meshgrid(np.linspace(0, width - 1, num=width),
                                 np.linspace(0, height - 1, num=height))

        # Process depth image from ray length to camera axis depth
        # distance = ((rows - center_y)**2 + (cols - center_x)**2)**0.5
        # points_z = img_depth / (1 + (distance / f) ** 2) ** 0.5
        points_z = img_depth
        # Create x and y position
        points_x = points_z * (cols - center_x) / f
        points_y = points_z * (rows - center_y) / f

        return points_x.reshape(-1), points_y.reshape(-1), points_z.reshape(-1)

    @staticmethod
    def rgb_to_float(img_color):
        """ Stack uint8 rgb image into a single float
            array (efficiently) for ros compatibility """
        r = np.ravel(img_color[:, :, 0]).astype(int)
        g = np.ravel(img_color[:, :, 1]).astype(int)
        b = np.ravel(img_color[:, :, 2]).astype(int)
        color = np.left_shift(r, 16) + np.left_shift(g, 8) + b
        packed = pack('%di' % len(color), *color)
        unpacked = unpack('%df' % len(color), packed)
        return np.array(unpacked)


if __name__ == '__main__':
    rospy.init_node('flat_data_player')
    flat_data_player = FlatDataPlayer()
    rospy.spin()