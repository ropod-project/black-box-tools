#!/usr/bin/env python3
import sys
import yaml
import os.path
import curses
import rospy

from black_box_tools.ros.black_box_rosbag import BlackBoxRosbag
from black_box_tools.db_utils import DBUtils

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
        return {'sync_time': True, 
                'time_step': 1.0, 
                'sleep_duration':0.5, 
                'db_name': 'logs'}

def get_desired_duration(start_time, stop_time) :
    duration = stop_time - start_time
    print("Length of Rosbag file:", duration)
    start_offset, stop_offset = 0.0, duration
    user_start = input("Enter start time. Press enter for default: ")
    if user_start != '' :
        try:
            start_offset = float(user_start)
        except Exception as e:
            print("Unable to convert to float. Using default")
    user_stop = input("Enter stop time. Press enter for default: ")
    if user_stop != '' :
        try:
            stop_offset = float(user_stop)
        except Exception as e:
            print("Unable to convert to float. Using default")
    return start_time+start_offset, start_time+stop_offset

def curses_func(stdscr) :
    """curses friendly function. All curses related functions are called here.
    This makes the rest of the code safe from curses mess ups (if any occurs:
    refer to  https://docs.python.org/3/library/curses.html#curses.wrapper)
    Pressing SpaceBar toggles between rosbag playing and pausing

    """
    stdscr.nodelay(True)
    start_time = rosbag.start_timestamp
    stop_time = rosbag.stop_timestamp
    duration = stop_time - start_time
    while rosbag.is_playing() and not rospy.is_shutdown():
        c = stdscr.getch()
        curses.flushinp()
        stdscr.clear()
        miny, minx = stdscr.getbegyx()
        maxy, maxx = stdscr.getmaxyx()
        pause_status = "PAUSED" if rosbag.sync.is_paused else "RUNNING"
        current_time = rosbag.sync.get_current_time()
        string = "[ "+pause_status+" ] Time: "+str(current_time)
        string_2 = "Rosbag time: "+str(current_time - start_time)+" / "+str(duration)
        stdscr.addstr(maxy-miny-3, 0, string)
        stdscr.addstr(maxy-miny-2, 0, string_2)
        if c == ord(' ') :
            rosbag.sync.toggle_pause()
        rospy.sleep(0.5)
        stdscr.refresh()

if __name__ == '__main__':
    rospy.init_node('rosbag_play')
    config_param = get_config_param()

    if len(sys.argv) < 2:
        rospy.logwarn('\nUsage: python3 rosbag_play.py [db_name]')
        rospy.logwarn('No database name provided. Using default.\n')
        db_name = config_param['db_name']
    else :
        db_name = sys.argv[1]

    start_time = DBUtils.get_db_oldest_timestamp(db_name)
    stop_time = DBUtils.get_db_newest_timestamp(db_name)
    start_time, stop_time = get_desired_duration(start_time, stop_time)
    rosbag = BlackBoxRosbag(
            db_name=db_name, 
            start_time=start_time,
            stop_time=stop_time,
            sync_time=config_param['sync_time'], 
            time_step=config_param['time_step'], 
            sleep_duration=config_param['sleep_duration'])
    try:
        rosbag.play()
        curses.wrapper(curses_func) # wait till rosbag finishes & show status
    except (KeyboardInterrupt, SystemExit):
        rospy.loginfo("Stopping rosbag play")
        rosbag.stop()
