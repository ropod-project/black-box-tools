import threading
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
        actual_start_time = DBUtils.get_db_oldest_doc(self.black_box_db_name)
        actual_stop_time = DBUtils.get_db_newest_doc(self.black_box_db_name)
        self.start_timestamp = kwargs.get('start_time', actual_start_time)
        self.stop_timestamp = kwargs.get('stop_time', actual_stop_time)
        if actual_start_time > self.start_timestamp or \
                actual_stop_time < self.stop_timestamp or \
                self.start_timestamp > self.stop_timestamp :
            print("WARNING: Incorrect start or stop time. Using default duration")
            self.start_timestamp = actual_start_time
            self.stop_timestamp = actual_stop_time

        self.topic_managers = []
        self.topic_manager_threads = []
        self.playing = False

        data_collections = DBUtils.get_data_collection_names(self.black_box_db_name)

        # create list of locks (each for one topic)
        self.locks = [threading.Lock() for collection in data_collections]

        # create syncronizer object and assign it to a thread
        self.sync = Syncronizer(
                self.start_timestamp, 
                self.locks, 
                time_step=self.time_step,
                sleep_duration=self.sleep_duration)
        self.sync_thread = threading.Thread(target=self.sync.increment_time)
        self.sync_thread.daemon = True

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
            data_thread = threading.Thread(
                    target=topic_manager.publish_data,
                    kwargs={'dict_msgs': data_cursor,
                           'sync_time': self.sync_time,
                           'global_clock_start': self.start_timestamp,
                           'lock': self.locks[i],
                           'sync': self.sync })
            data_thread.daemon = True
            self.topic_manager_threads.append(data_thread)

    def play(self):
        '''Takes the collections in the black box database which contain data
        that can be mapped to ROS messages and publishes the data on the appropriate
        topics (as specified in the black box metadata).

        '''
        print('[black_box_rosbag] Starting replay')
        self.playing = True
        for data_thread in self.topic_manager_threads:
            data_thread.start()
        self.sync_thread.start()

    def stop_playing(self):
        '''Interrupts the process of black box data playing.
        '''
        print('[black_box_rosbag] Stopping replay')
        self.playing = False
        for topic_manager in self.topic_managers:
            topic_manager.stop_publishing()

        for data_thread in self.topic_manager_threads:
            data_thread.join()
        self.sync_thread.join()

    def is_playing(self):
        '''Returns True if data is being played back; returns False otherwise.
        '''
        return self.playing and self.sync.get_current_time() < self.stop_timestamp
