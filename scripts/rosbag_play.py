#!/usr/bin/env python3
import sys
import rospy
from black_box_tools.ros.black_box_rosbag import BlackBoxRosbag

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python rosbag_play.py [db_name]')
    db_name = sys.argv[1]

    rospy.init_node('rosbag_play')
    rosbag = BlackBoxRosbag()
    try:
        rosbag.play(db_name)
        while rosbag.is_playing() and not rospy.is_shutdown():
            rospy.sleep(0.05)
    except (KeyboardInterrupt, SystemExit):
        rosbag.stop_playing()
