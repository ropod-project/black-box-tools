from importlib import import_module
import pymongo as pm
from rospy_message_converter.message_converter import convert_dictionary_to_ros_message
import rospy

class TopicUtils(object):
    '''Interface that takes care of publishing black box data to a ROS topic.

    Constructor arguments:
    @param topic -- name of the topic to which to publish data
    @param msg_type -- name of the underlying message type (e.g. geometry_msgs/Twist)
    @param direct_msg_mapping -- indicates whether incoming data are directly
                                 mapped to a ROS message type (holds true
                                 if ROS data are logged) (default True)
    @param queue_size -- queue size for the message publisher (default 1)

    Note: Only a direct mapping between black box data and ROS messages is currently supported.

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, topic, msg_type_name, direct_msg_mapping=True, queue_size=1):
        self.topic = topic
        self.msg_type_name = msg_type_name
        self.direct_msg_mapping = direct_msg_mapping
        self.queue_size = queue_size

        msg_type = self.__get_msg_type(self.msg_type_name)
        self.topic_pub = rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def publish_data(self, dict_msgs, sync_time=True):
        '''Publishes a list of black box data items to self.topic. If "sync_time"
        is set to True, synchronises the messages based on the timestamps
        of the data items.

        @param dict_msgs -- either a list of black box data item dictionaries
                            or a pymongo.cursor.Cursor instance for retrieving
                            data item dictionaries
        @param sync_time -- indicates whether the time between the published
                            messages should be synchronised based on the
                            message timestamps (default True)

        '''
        if isinstance(dict_msgs, list):
            if dict_msgs:
                current_msg_time = dict_msgs[0]['timestamp']
                for i in range(len(dict_msgs)-1):
                    self.publish_dict(dict_msgs[i])
                    if sync_time:
                        # we find the delay between two consecutive messages and sleep
                        next_msg_time = dict_msgs[i+1]['timestamp']
                        msg_time_delta = next_msg_time - current_msg_time
                        print('Sleeping for {0}s'.format(msg_time_delta))
                        rospy.sleep(msg_time_delta)
                        current_msg_time = next_msg_time
                self.publish_dict(dict_msgs[-1])
            else:
                print('[publish_data] dict_msgs empty; nothing to publish')
        elif isinstance(dict_msgs, pm.cursor.Cursor):
            dict_msg = {}
            try:
                dict_msg = next(dict_msgs)
                current_msg_time = None
                if dict_msg:
                    current_msg_time = dict_msg['timestamp']

                while dict_msg:
                    self.publish_dict(dict_msg)
                    if sync_time:
                        # we find the delay between two consecutive messages and sleep
                        dict_msg = next(dict_msgs)
                        next_msg_time = dict_msg['timestamp']
                        msg_time_delta = next_msg_time - current_msg_time
                        print('Sleeping for {0}s'.format(msg_time_delta))
                        rospy.sleep(msg_time_delta)
                        current_msg_time = next_msg_time
            except StopIteration:
                if dict_msg:
                    self.publish_dict(dict_msg)
        else:
            print('[publish_data] dict_msgs has to be either a list or a pymongo.cursor.Cursor')

    def publish_dict(self, dict_msg):
        '''Publishes the given item to self.topic.

        @param dict_msg -- a black box data item dictionary

        '''
        msg = self.__dict_to_msg(dict_msg)
        if msg:
            self.topic_pub.publish(msg)
        else:
            print('[publish_dict] The input dictionary could not be converted to a ROS message')

    def __dict_to_msg(self, dict_msg):
        '''Converts the input dictionary to a ROS message of type "self.msg_type_name".

        @param dict_msg -- a black box data item dictionary

        '''
        msg = None
        if self.direct_msg_mapping:
            msg = convert_dictionary_to_ros_message(self.msg_type_name, dict_msg)
        else:
            # TODO: define a procedure to map indirect message correspondences to ROS messages
            pass
        return msg

    def __get_msg_type(self, msg_type_name):
        '''Returns a "type" corresponding to the type of the input message.

        @param msg_type_name -- name of a message type (e.g. geometry_msgs/Twist)

        '''
        msg_pkg_name, msg_type_name = msg_type_name.split('/')
        msg_pkg_name = '{0}.msg'.format(msg_pkg_name)
        msg_pkg = import_module(msg_pkg_name)
        msg_type = getattr(msg_pkg, msg_type_name)
        return msg_type
