import threading
from black_box_tools.db_utils import DBUtils
from black_box_tools.ros.topic_utils import TopicUtils
from black_box_tools.ros.syncronizer import Syncronizer

class BlackBoxRosbag(object):
    '''Mimics the playback functionality of a rosbag for black box data.

    @author Alex Mitrevski, Dharmin B.
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self):
        self.topic_managers = []
        self.topic_manager_threads = []
        self.playing = False

    def play(self, black_box_db_name, sync_time=True):
        '''Takes the collections in the black box database which contain data
        that can be mapped to ROS messages and publishes the data on the appropriate
        topics (as specified in the black box metadata).

        @param black_box_db_name -- name of a black box database
        @param sync_time -- indicates whether the time between the published
                            messages should be synchronised based on the
                            message timestamps (default True)

        '''
        data_collections = DBUtils.get_data_collection_names(black_box_db_name)
        self.topic_managers = []
        self.topic_manager_threads = []
        self.locks = []

        # we get the timestamp of the oldest document among the collections
        # so that we can synchronise the published data
        start_timestamp = 1e20
        stop_time = 10
        current_time = 0
        for collection in data_collections:
            oldest_doc = DBUtils.get_oldest_doc(black_box_db_name, collection)
            if oldest_doc['timestamp'] < start_timestamp:
                start_timestamp = oldest_doc['timestamp']

        # create list of locks (each for one topic)
        self.locks = [threading.Lock() for collection in data_collections]
        # create syncronizer object and assign it to a thread
        sync = Syncronizer(start_timestamp, self.locks, sleep_duration=0.8)
        sync_thread = threading.Thread(target=sync.increment_time)
        sync_thread.daemon = True

        # create topic_utils object and assign it to a thread for each topic
        for i, collection in enumerate(data_collections):
            collection_metadata = DBUtils.get_collection_metadata(black_box_db_name, collection)
            topic_manager = TopicUtils(
                    collection_metadata['ros']['topic_name'],
                    collection_metadata['ros']['msg_type'],
                    collection_metadata['ros']['direct_msg_mapping'])
            self.topic_managers.append(topic_manager)

            data_cursor = DBUtils.get_doc_cursor(black_box_db_name, collection)
            data_thread = threading.Thread(
                    target=topic_manager.publish_data,
                    kwargs={'dict_msgs': data_cursor,
                           'sync_time': sync_time,
                           'global_clock_start': start_timestamp,
                           'lock': self.locks[i],
                           'sync': sync })
            data_thread.daemon = True
            self.topic_manager_threads.append(data_thread)

        print('[black_box_rosbag] Starting replay')
        self.playing = True
        for data_thread in self.topic_manager_threads:
            data_thread.start()
        sync_thread.start()

    def stop_playing(self):
        '''Interrupts the process of black box data playing.
        '''
        print('[black_box_rosbag] Stopping replay')
        self.playing = False
        for topic_manager in self.topic_managers:
            topic_manager.stop_publishing()

        for data_thread in self.topic_manager_threads:
            data_thread.join()

    def is_playing(self):
        '''Returns True if data is being played back; returns False otherwise.
        '''
        return self.playing
