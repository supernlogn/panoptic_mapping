#!/usr/bin/env python

import os
import csv
import rospy
from rosgraph_msgs.msg import Clock
import time


class SimTimePublisher:
    def __init__(self):
        """ This node publishes times from a timestamps file to /clock topic.
        The timestamps published are not the exact ones, but they are divided by a play_rate coefficient.
        The timestamps file is provided by the datasets that the simulation runs in.
        """
        self.play_rate = rospy.get_param('~play_rate', 5.0)  # Hz
        self.max_frames = rospy.get_param('~max_frames', 1e9)
        self.refresh_rate = rospy.get_param('~refresh_rate', 5.0)  # Hz
        self.data_path = self.data_path = rospy.get_param(
            '~data_path', '/home/ioannis/datasets/flat_dataset/run1')
        self.stamps_file_path = os.path.join(self.data_path, 'timestamps.csv')
        self.times = []
        self.setup_times()
        self.clock_pub = rospy.Publisher('/clock',
                                         Clock,
                                         queue_size=2 * len(self.times))
        self.sim_clock = Clock()
        self.index = 0

    def setup_times(self):
        self.times = []
        if not os.path.isfile(self.stamps_file_path):
            rospy.logfatal("No timestamp file '%s' found." %
                           self.stamps_file_path)
        with open(self.stamps_file_path, 'r') as read_obj:
            csv_reader = csv.reader(read_obj)
            for row in csv_reader:
                if row[0] == "ImageID":
                    continue
                self.times.append(float(row[1]) / 1e9)
        self.times = sorted(self.times)
        self.times = [(x - self.times[0]) / self.play_rate for x in self.times]
        rospy.loginfo(self.times)

    def start(self, _):
        lenTimes = len(self.times)
        while (not rospy.is_shutdown()):
            self.sim_clock.clock = rospy.Time.from_sec(self.times[self.index])
            self.clock_pub.publish(self.sim_clock)
            self.index += 1
            if (self.index >= lenTimes):  # if end is reached then stay there
                self.index = lenTimes - 1
                time.sleep(1)  # sleep in wall time
            else:
                time.sleep(1 / self.refresh_rate)  # sleep in wall time

def main():
    rospy.init_node('sim_time_publisher', anonymous=True)
    sim_time_publisher = SimTimePublisher()
    sim_time_publisher.start(None)

if __name__ == '__main__':
    main()