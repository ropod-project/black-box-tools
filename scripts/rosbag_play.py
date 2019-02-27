#!/usr/bin/env python3
import sys
import yaml
import os.path
import rospy

from black_box_tools.ros.black_box_rosbag import BlackBoxRosbag

def get_config_param() :
    code_dir = os.path.abspath(os.path.dirname(__file__))
    main_dir = os.path.dirname(code_dir)
    config_file = os.path.join(main_dir, "config/config.yaml")
    try:
        file_handle = open(config_file, 'r')
        data = yaml.load(file_handle)
        file_handle.close()
        return data
    except Exception as e:
        return {'sync_time': True, 'time_step': 1.0, 'sleep_duration':0.5}

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python rosbag_play.py [db_name]')
        db_name = "logs"
    else :
        db_name = sys.argv[1]

    config_param = get_config_param()

    rospy.init_node('rosbag_play')
    rosbag = BlackBoxRosbag(
            db_name=db_name, 
            sync_time=config_param['sync_time'], 
            time_step=config_param['time_step'], 
            sleep_duration=config_param['sleep_duration'])
    try:
        rosbag.play()
        while rosbag.is_playing() and not rospy.is_shutdown():
            rospy.sleep(0.05)
    except (KeyboardInterrupt, SystemExit):
        rosbag.stop_playing()
