# import threading
import multiprocessing
from black_box_tools.db_utils import DBUtils
from black_box_tools.ros.topic_utils import TopicUtils
from black_box_tools.ros.syncronizer import Syncronizer

class BlackBoxRosbag(object):
    '''Mimics the playback functionality of a rosbag for black box data.

    @param db_name -- name of a black box database
    @param sync_time -- indicates whether the time between the published
                        messages should be synchronised based on the
                        message timestamps (default True)


    @author Alex Mitrevski, Dharmin B.
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, **kwargs) :
        self.black_box_db_name = kwargs.get('db_name', 'logs')
        self.sync_time = kwargs.get('sync_time', True)
        self.time_step = kwargs.get('time_step', 1.0)
        self.sleep_duration = kwargs.get('sleep_duration', 0.5)
        actual_start_time = DBUtils.get_db_oldest_timestamp(self.black_box_db_name)
        actual_stop_time = DBUtils.get_db_newest_timestamp(self.black_box_db_name)
        self.start_timestamp = kwargs.get('start_time', actual_start_time)
        self.stop_timestamp = kwargs.get('stop_time', actual_stop_time)
        if actual_start_time > self.start_timestamp or \
                actual_stop_time < self.stop_timestamp or \
                self.start_timestamp > self.stop_timestamp :
            print("WARNING: Incorrect start or stop time. Using default duration")
            self.start_timestamp = actual_start_time
            self.stop_timestamp = actual_stop_time
        self.current_time = self.start_timestamp

        self.status = "PAUSED"

        self.topic_managers = []
        self.topic_manager_threads = []

        data_collections = DBUtils.get_data_collection_names(self.black_box_db_name)

        # create list of locks for syncing (each for one topic)
        self.locks = [multiprocessing.Lock() for collection in data_collections]
        # create list of queues to access global current time (maintained by syncronizer)
        self.queues = [multiprocessing.Queue() for collection in data_collections]
        self.queues.append(multiprocessing.Queue()) # for current(parent) process

        sync_pause_conn, self.rosbag_pause_conn = multiprocessing.Pipe(duplex=True)

        # create syncronizer object and assign it to a thread
        self.sync = Syncronizer(
                self.start_timestamp, 
                self.locks, 
                self.queues,
                sync_pause_conn,
                time_step=self.time_step,
                sleep_duration=self.sleep_duration)
        self.sync_thread = multiprocessing.Process(target=self.sync.increment_time, daemon=True)

        # create topic_utils object and assign it to a thread for each topic
        for i, collection in enumerate(data_collections):
            collection_metadata = DBUtils.get_collection_metadata(self.black_box_db_name, collection)
            topic_manager = TopicUtils(
                    collection_metadata['ros']['topic_name'],
                    collection_metadata['ros']['msg_type'],
                    collection_metadata['ros']['direct_msg_mapping'])
            self.topic_managers.append(topic_manager)

            data_cursor = DBUtils.get_doc_cursor(
                    self.black_box_db_name, 
                    collection, 
                    self.start_timestamp, 
                    self.stop_timestamp)
            data_thread = multiprocessing.Process(
                    target=topic_manager.publish_data,
                    kwargs={'dict_msgs': data_cursor,
                           'sync_time': self.sync_time,
                           'global_clock_start': self.start_timestamp,
                           'lock': self.locks[i],
                           'queue': self.queues[i]},
                    daemon=True)
            # data_thread.daemon = True
            self.topic_manager_threads.append(data_thread)
        for data_thread in self.topic_manager_threads:
            data_thread.start()
        self.sync_thread.start()

    def play(self):
        """Start/resume the playing of the rosbag messages. It sends a message to the 
        Syncronizer process. Syncronizer gets out of a while loop and starts 
        normal fucntioning until a pause message is sent or process is stopped.

        :returns None

        """
        print('[black_box_rosbag] Starting replay')
        self.rosbag_pause_conn.send((True, False))
        self.status = "RUNNING"

    def stop(self):
        """Interrupts the process of black box data playing.

        :returns: None
        """
        print('[black_box_rosbag] Stopping replay')
        self.rosbag_pause_conn.send((False, False))

        for data_thread in self.topic_manager_threads:
            data_thread.join()
        self.sync_thread.join()

    def pause(self):
        """Pause the playing of the rosbag messages. It sends a message to the 
        Syncronizer process. Syncronizer goes into a while loop until it receives
        another message

        :returns: None

        """
        self.rosbag_pause_conn.send((True, True))
        self.status = "PAUSED"

    def get_current_time(self):
        """Get current time from queue if available.
        Otherwise return the class variable
        :returns: Float (global current time)

        """
        while not self.queues[-1].empty():
            self.current_time = self.queues[-1].get()
        return self.current_time

    def is_playing(self):
        """Returns True if data is being played back; returns False otherwise.

        :returns: bool (if the rosbag is playing or not) 
        NOTE: Not to confuse with playing/paused. That is self.status

        """
        return self.current_time <= self.stop_timestamp
