import threading
from black_box_tools.db_utils import DBUtils
from black_box_tools.ros.topic_utils import TopicUtils

class BlackBoxRosbag(object):
    '''Mimics the playback functionality of a rosbag for black box data.

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    @staticmethod
    def play(black_box_db_name, sync_time=True):
        '''Takes the collections in the black box database which contain data
        that can be mapped to ROS messages and publishes the data on the appropriate
        topics (as specified in the black box metadata -- this is still a TODO).

        @param black_box_db_name -- name of a black box database
        @param sync_time -- indicates whether the time between the published
                            messages should be synchronised based on the
                            message timestamps (default True)

        '''
        data_collections = DBUtils.get_data_collection_names(black_box_db_name)
        topic_managers = []
        topic_manager_threads = []
        for collection in data_collections:
            collection_metadata = DBUtils.get_collection_metadata(black_box_db_name, collection)
            topic_name = collection_metadata['ros']['topic_name']
            msg_type = collection_metadata['ros']['msg_type']
            direct_msg_mapping = collection_metadata['ros']['direct_msg_mapping']
            topic_manager = TopicUtils(topic_name, msg_type, direct_msg_mapping)
            topic_managers.append(topic_manager)

            data_cursor = DBUtils.get_doc_cursor(black_box_db_name, collection)
            data_thread = threading.Thread(target=topic_manager.publish_data,
                                           kwargs={'dict_msgs': data_cursor,
                                                   'sync_time': sync_time})
            topic_manager_threads.append(data_thread)

        print('Starting replay')
        for data_thread in topic_manager_threads:
            data_thread.start()
