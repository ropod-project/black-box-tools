#! /usr/bin/env python3

import time
import os
import unittest
import pymongo as pm

from black_box_tools.db_utils import DBUtils

class TestDBUtils(unittest.TestCase):

    """ Tests for the functions defined in DBUtils class in db_utils.py"""

    @classmethod
    def setUpClass(cls):
        cls.test_db_name = "bb_tools_test_data"
        test_dir = os.path.abspath(os.path.dirname(__file__))
        cls.test_db_dir = os.path.join(test_dir, cls.test_db_name)

        host = 'localhost'
        port = 27017
        if 'DB_HOST' in os.environ:
            host = os.environ['DB_HOST']
        if 'DB_PORT' in os.environ:
            port = int(os.environ['DB_PORT'])
        cls.client = pm.MongoClient(host=host, port=port)

    @classmethod
    def tearDownClass(cls):
        pass

    def test_get_data_collection_names(self):
        collection_names = DBUtils.get_data_collection_names(db_name=self.test_db_name)
        self.assertListEqual(collection_names, ['ros_sw_ethercat_parser_data', 'ros_ropod_cmd_vel'])

    def test_get_all_docs(self):
        docs = DBUtils.get_all_docs(db_name=self.test_db_name,
                                    collection_name='ros_ropod_cmd_vel')
        self.assertEqual(len(docs), 149)

    def test_get_docs(self):
        start_time = 1544441409.88607
        stop_time = 1544441411.28594
        docs = DBUtils.get_docs(db_name=self.test_db_name,
                                collection_name='ros_ropod_cmd_vel',
                                start_time=start_time,
                                stop_time=stop_time)
        self.assertEqual(len(docs), 9)
        for doc in docs:
            self.assertLessEqual(doc['timestamp'], stop_time)
            self.assertGreaterEqual(doc['timestamp'], start_time)

    def test_get_doc_cursor(self):
        start_time = 1544441409.88607
        stop_time = 1544441411.28594
        doc_cursor = DBUtils.get_doc_cursor(db_name=self.test_db_name,
                                            collection_name='ros_ropod_cmd_vel',
                                            start_time=start_time,
                                            stop_time=stop_time)
        docs = [doc for doc in doc_cursor]
        self.assertEqual(len(docs), 9)
        for doc in docs:
            self.assertLessEqual(doc['timestamp'], stop_time)
            self.assertGreaterEqual(doc['timestamp'], start_time)

    def test_get_docs_of_last_n_secs(self):
        docs = DBUtils.get_docs_of_last_n_secs(db_name=self.test_db_name,
                                               collection_name='ros_ropod_cmd_vel',
                                               n=2)
        self.assertEqual(len(docs), 6)

    def test_get_collection_metadata(self):
        doc = DBUtils.get_collection_metadata(db_name=self.test_db_name,
                                              collection_name='ros_ropod_cmd_vel')
        del doc['_id']
        self.assertDictEqual(
            doc,
            {
                'collection_name': 'ros_ropod_cmd_vel',
                'ros': {
                    'msg_type': 'geometry_msgs/Twist',
                    'direct_msg_mapping': True,
                    'topic_name': '/ropod/cmd_vel'}
            })

    def test_get_oldest_doc(self):
        doc = DBUtils.get_oldest_doc(db_name=self.test_db_name,
                                     collection_name='ros_ropod_cmd_vel')
        del doc['_id']
        self.assertDictEqual(
            doc,
            {'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
             'linear': {'x': -0.0, 'y': -0.0, 'z': 0.0},
             'timestamp': 1544441408.3859708})

    def test_get_newest_doc(self):
        doc = DBUtils.get_newest_doc(db_name=self.test_db_name,
                                     collection_name='ros_ropod_cmd_vel')
        del doc['_id']
        self.assertDictEqual(
            doc,
            {'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
             'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
             'timestamp': 1544441432.830365})

    def test_get_db_oldest_timestamp(self):
        timestamp = DBUtils.get_db_oldest_timestamp(db_name=self.test_db_name)
        self.assertEqual(timestamp, 1544441408.0746574)

    def test_get_db_newest_timestamp(self):
        timestamp = DBUtils.get_db_newest_timestamp(db_name=self.test_db_name)
        self.assertEqual(timestamp, 1544441433.7863772)


if __name__ == '__main__':
    unittest.main()
