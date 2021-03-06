#!/usr/bin/env python3
import sys
import yaml
import os.path
import curses
import rospy
import time
import argparse
from termcolor import colored

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
                'event_playback_duration': 3,
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
            print(colored("Unable to convert to float. Using default", 'red'))
    user_stop = input("Enter stop time. Press enter for default: ")
    if user_stop != '' :
        try:
            stop_offset = float(user_stop)
        except Exception as e:
            print(colored("Unable to convert to float. Using default", 'red'))
    return start_time+start_offset, start_time+stop_offset

def choose_event(events, default_event_num=1):
    """Ask user to choose an event from a list of events.

    :events: list of dicts (events dictionaries)
    :default_event_num: int (default event num if user does not input anything)
    :returns: int (index of event chosen by the user)

    """
    print('List of events')
    for i, event in enumerate(events) :
        print(i+1,"-",event['description'], "(", event['timestamp'], ")")
    user_inp = input("Choose an event from above (exit:0)(default:"+str(default_event_num)+") : ")
    if user_inp is None :
        chosen_event = default_event_num
    if user_inp.isnumeric() :
        chosen_event = int(user_inp)
        if chosen_event > len(events) or chosen_event < 0 :
            print(colored("Invalid event selected. Using default.", 'red'))
            chosen_event = default_event_num
    else :
        print(colored("Invalid event selected. Using default.", 'red'))
        chosen_event = default_event_num
    return chosen_event

def play_rosbag(start_time, stop_time):
    """create a Rosbag object and call its play function.

    :start_time: float
    :stop_time: float
    :returns: None

    """
    rosbag = BlackBoxRosbag(
            db_name=db_name, 
            start_time=start_time,
            stop_time=stop_time,
            sync_time=config_param['sync_time'], 
            time_step=config_param['time_step'], 
            sleep_duration=config_param['sleep_duration'])
    try:
        rosbag.play()
        curses.wrapper(curses_func, rosbag) # wait till rosbag finishes & show status
    except (KeyboardInterrupt, SystemExit):
        print(colored("Interrupting rosbag play", 'red'))
    rosbag.stop()

def curses_func(stdscr, rosbag) :
    """curses friendly function. All curses related functions are called here.
    This makes the rest of the code safe from curses mess ups (if any occurs:
    refer to  https://docs.python.org/3/library/curses.html#curses.wrapper)
    Pressing SpaceBar toggles between rosbag playing and pausing

    """
    stdscr.nodelay(True)
    start_time = rosbag.start_timestamp
    stop_time = rosbag.stop_timestamp
    duration = stop_time - start_time
    while rosbag.is_playing() :
        c = stdscr.getch()
        curses.flushinp()
        stdscr.clear()
        miny, minx = stdscr.getbegyx()
        maxy, maxx = stdscr.getmaxyx()
        current_time = rosbag.get_current_time()
        string = "[ "+rosbag.status+" ] Time: "+str(current_time)
        string_2 = "Rosbag time: "+str(current_time - start_time)+" / "+str(duration)
        stdscr.addstr(maxy-miny-3, 0, string)
        stdscr.addstr(maxy-miny-2, 0, string_2)
        if c == ord(' ') :
            if rosbag.status == "PAUSED" :
                rosbag.play()
            else :
                rosbag.pause()
        time.sleep(config_param['sleep_duration'])
        stdscr.refresh()

if __name__ == '__main__':
    # rospy.init_node('rosbag_play')
    config_param = get_config_param()

    run_events = False

    parser = argparse.ArgumentParser(description="Play rosbag from mongo db")
    parser.add_argument('-db', help='name of the mongo db', default=config_param['db_name'])
    parser.add_argument('-e', '--events', action='store_true', help='Enable events playback')
    args = parser.parse_args()
    db_name = args.db

    if args.events and 'ros_ropod_event' in DBUtils.get_data_collection_names(db_name) \
        and len(DBUtils.get_all_docs(db_name, 'ros_ropod_event')) > 0:
        events = DBUtils.get_all_docs(db_name, 'ros_ropod_event')
        chosen_event_index = choose_event(events, 1)
        while chosen_event_index != 0 :
            chosen_event = events[chosen_event_index-1]
            event_time = chosen_event['timestamp']
            play_rosbag(event_time-config_param['event_playback_duration'], 
                    event_time+config_param['event_playback_duration'])
            chosen_event_index = choose_event(events, chosen_event_index)

    else :
        start_time = DBUtils.get_db_oldest_timestamp(db_name)
        stop_time = DBUtils.get_db_newest_timestamp(db_name)
        start_time, stop_time = get_desired_duration(start_time, stop_time)
        play_rosbag(start_time, stop_time)
