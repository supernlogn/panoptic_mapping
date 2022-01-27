#!/usr/bin/env python
import os
import json
import csv

import tf
import rospy
from rospy.timer import sleep
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
from std_srvs.srv import EmptyResponse
from panoptic_mapping_msgs.msg import DetectronLabel, DetectronLabels
from PIL import Image as PilImage

import numpy as np
import cv2


class FlatDataPlayer(object):
    def __init__(self):
        """  Initialize ros node and read params """
        # params
        self.data_path = rospy.get_param(
            '~data_path', '/home/ioannis/datasets/flat_dataset/run1')
        self.global_frame_name = rospy.get_param('~global_frame_name', 'world')
        self.sensor_frame_name = rospy.get_param('~sensor_frame_name',
                                                 "depth_cam")
        self.use_detectron = rospy.get_param('~use_detectron', False)
        self.play_rate = rospy.get_param('~play_rate', 1.0)
        self.wait = rospy.get_param('~wait', False)
        self.max_frames = rospy.get_param('~max_frames', 1e9)
        self.refresh_rate = rospy.get_param('~refresh_rate', 1)  # Hz
        # drift generator
        self.use_noise = rospy.get_param("~use_noise")
        # ROS
        self.color_pub = rospy.Publisher("~color_image", Image, queue_size=100)
        self.depth_pub = rospy.Publisher("~depth_image", Image, queue_size=100)
        self.id_pub = rospy.Publisher("~id_image", Image, queue_size=100)
        if self.use_detectron:
            self.label_pub = rospy.Publisher("~labels",
                                             DetectronLabels,
                                             queue_size=100)
        if self.use_noise:
            self.pose_pub = rospy.Publisher("~pose",
                                            TransformStamped,
                                            queue_size=100)
        else:
            self.pose_pub = rospy.Publisher("~pose",
                                            PoseStamped,
                                            queue_size=100)

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
        sleep(10)
        # if self.wait:
        #     self.start_srv = rospy.Service('~start', Empty, self.start)
        # else:
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
        files = [color_file, depth_file, pose_file]
        if self.use_detectron:
            pred_file = file_id + "_predicted.png"
            labels_file = file_id + "_labels.json"
            files += [pred_file, labels_file]
        else:
            pred_file = file_id + "_segmentation.png"
            files.append(pred_file)
        for f in files:
            if not os.path.isfile(f):
                rospy.logwarn("Could not find file '%s', skipping frame." % f)
                self.current_index += 1
                return

        # Load and publish Color image.
        cv_img = cv2.imread(color_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.sensor_frame_name
        self.color_pub.publish(img_msg)

        # Load and publish ID image.
        cv_img = cv2.imread(pred_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img[:, :, 0], "8UC1")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.sensor_frame_name
        self.id_pub.publish(img_msg)

        # Load and publish labels.
        if self.use_detectron:
            label_msg = DetectronLabels()
            label_msg.header.stamp = now
            with open(labels_file) as json_file:
                data = json.load(json_file)
                for d in data:
                    if 'instance_id' not in d:
                        d['instance_id'] = 0
                    if 'score' not in d:
                        d['score'] = 0
                    label = DetectronLabel()
                    label.id = d['id']
                    label.instance_id = d['instance_id']
                    label.is_thing = d['isthing']
                    label.category_id = d['category_id']
                    label.score = d['score']
                    label_msg.labels.append(label)
            self.label_pub.publish(label_msg)

        # Load and publish depth image. These are optional.
        cv_img = PilImage.open(depth_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(np.array(cv_img), "32FC1")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.sensor_frame_name
        self.depth_pub.publish(img_msg)

        # Load and publish transform.
        if os.path.isfile(pose_file):
            pose_data = [float(x) for x in open(pose_file, 'r').read().split()]
            transform = np.eye(4)
            for row in range(4):
                for col in range(4):
                    transform[row, col] = pose_data[row * 4 + col]
            rotation = tf.transformations.quaternion_from_matrix(transform)
            position = transform[:, 3]

        if self.use_noise:
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
            pose_msg.child_frame_id = self.sensor_frame_name
            self.pose_pub.publish(pose_msg)
        else:
            self.tf_broadcaster.sendTransform(position, rotation, now,
                                              self.sensor_frame_name,
                                              self.global_frame_name)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = self.global_frame_name
            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]
            pose_msg.pose.position.z = position[2]

            pose_msg.pose.orientation.x = rotation[0]
            pose_msg.pose.orientation.y = rotation[1]
            pose_msg.pose.orientation.z = rotation[2]
            pose_msg.pose.orientation.w = rotation[3]
            self.pose_pub.publish(pose_msg)

        self.current_index += 1
        if self.current_index > self.max_frames:
            rospy.signal_shutdown("Played reached max frames (%i)" %
                                  self.max_frames)


if __name__ == '__main__':
    rospy.init_node('flat_data_player')
    flat_data_player = FlatDataPlayer()
    rospy.spin()
