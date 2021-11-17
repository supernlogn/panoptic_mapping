#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Clock
import time


def signal_handler(signum, frame):
    raise rospy.ROSInterruptException

class SimTimeTalker:
    def __init__(self):
        """ This class publishes time to /clock topic. 
        Essentially it creates a loop which iterates every 1/refresh_rate seconds
        and it multiplies the passed time by a sim_speed_multiplier factor.
        So if sim_speed_multiplier is 2, time passes twice as fast
        """
        self.rate = rospy.get_param('~refresh_rate', 200.0) # 5hz
        self.sim_speed_multiplier = rospy.get_param('~sim_speed_multiplier', 1.0)
        self.pub = rospy.Publisher('clock', Clock, queue_size=100)

    def start(self):
        """ This starts the main loop to publish times to /clock topic.
        The loop stops only after a ctrl+C or a shutdown signal is received.
        """
        sim_clock = Clock()
        zero_time = time.time()
        sleep_time = 1.0 / self.rate
        while not rospy.is_shutdown():
            sim_clock.clock = rospy.Time.from_sec(self.sim_speed_multiplier*(time.time() - zero_time))
            self.pub.publish(sim_clock)
            time.sleep(sleep_time)


def main():
    rospy.init_node('SimTimeTalker', anonymous=True)
    sim_time_talker = SimTimeTalker()
    try:
        sim_time_talker.start()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()