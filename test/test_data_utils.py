#! /usr/bin/env python3

import time
import os
import numpy as np
import subprocess
import unittest
import pymongo as pm

from black_box_tools.data_utils import DataUtils

class TestDataUtils(unittest.TestCase):

    """ Tests for the functions defined in DataUtils class in data_utils.py"""

    @classmethod
    def setUpClass(cls):
        cls.test_db_name = "bb_tools_test_data"
        test_dir = os.path.abspath(os.path.dirname(__file__))
        cls.test_db_dir = os.path.join(test_dir, cls.test_db_name)
        cls.collection_name = 'ros_ropod_cmd_vel'

        host, port = cls._get_db_host_and_port()
        cls.client = pm.MongoClient(host=host, port=port)
        success = cls._restore_test_db()
        assert(success)

    @classmethod
    def tearDownClass(cls):
        cls._drop_test_db()

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_get_all_measurement(self):
        database = self.client[self.test_db_name]
        collection = database[self.collection_name]
        doc_cursor = collection.find({})
        docs = [doc for doc in doc_cursor]
        measurements = DataUtils.get_all_measurements(docs, 'linear/x')
        self.assertEqual(measurements.shape, (149,))

    def test_get_variable_list(self):
        database = self.client[self.test_db_name]
        collection = database[self.collection_name]
        variable_list = DataUtils.get_variable_list(self.collection_name, collection)
        self.assertCountEqual(variable_list, ['ros_ropod_cmd_vel/linear/x', 'ros_ropod_cmd_vel/linear/y', 'ros_ropod_cmd_vel/linear/z', 'ros_ropod_cmd_vel/angular/x', 'ros_ropod_cmd_vel/angular/y', 'ros_ropod_cmd_vel/angular/z'])

    def test_expand_var_names(self):
        var_names = DataUtils.expand_var_names(['var1/*'], 2)
        self.assertListEqual(var_names, ['var1/0', 'var1/1'])

    def test_get_bb_query_msg_template(self):
        query_msg = DataUtils.get_bb_query_msg_template()
        self.assertIsInstance(query_msg, dict)
        self.assertIn('header', query_msg)
        self.assertIsInstance(query_msg['header'], dict)
        self.assertIn('metamodel', query_msg['header'])
        self.assertEqual('ropod-msg-schema.json', query_msg['header']['metamodel'])
        self.assertIn('payload', query_msg)
        self.assertIsInstance(query_msg['payload'], dict)

    def test_get_bb_query_msg(self):
        query_msg = DataUtils.get_bb_query_msg(
            sender_id='some_unique_id',
            bb_id='black_box_101',
            variable_list=['variable_name'],
            start_query_time=5.0,
            end_query_time=6.0)

        self.assertIsInstance(query_msg, dict)
        self.assertIn('header', query_msg)
        self.assertIsInstance(query_msg['header'], dict)
        self.assertIn('type', query_msg['header'])
        self.assertEqual('DATA-QUERY', query_msg['header']['type'])
        self.assertIn('timestamp', query_msg['header'])
        self.assertIn('payload', query_msg)
        self.assertIsInstance(query_msg['payload'], dict)
        self.assertEqual('some_unique_id', query_msg['payload'].get('senderId', None))
        self.assertEqual('black_box_101', query_msg['payload'].get('blackBoxId', None))
        self.assertListEqual(['variable_name'], query_msg['payload'].get('variables', None))
        self.assertEqual(5.0, query_msg['payload'].get('startTime', None))
        self.assertEqual(6.0, query_msg['payload'].get('endTime', None))

    def test_get_bb_latest_data_query_msg(self):
        query_msg = DataUtils.get_bb_latest_data_query_msg(
            sender_id='some_unique_id',
            bb_id='black_box_101',
            variable_list=['variable_name'])

        self.assertIsInstance(query_msg, dict)
        self.assertIn('header', query_msg)
        self.assertIsInstance(query_msg['header'], dict)
        self.assertIn('type', query_msg['header'])
        self.assertEqual('LATEST-DATA-QUERY', query_msg['header']['type'])
        self.assertIn('timestamp', query_msg['header'])
        self.assertIn('payload', query_msg)
        self.assertIsInstance(query_msg['payload'], dict)
        self.assertEqual('some_unique_id', query_msg['payload'].get('senderId', None))
        self.assertEqual('black_box_101', query_msg['payload'].get('blackBoxId', None))
        self.assertListEqual(['variable_name'], query_msg['payload'].get('variables', None))

    def test_parse_bb_variable_msg(self):
        response_msg = {
            'payload':
                {'variableList':
                    {'ros':
                        ['ros_cmd_vel/linear/x', 'ros_cmd_vel/linear/y']
                        }
                    }
                }

        response = DataUtils.parse_bb_variable_msg(response_msg)
        self.assertIsInstance(response, dict)
        self.assertIn('ros_cmd_vel', response)
        self.assertIsInstance(response['ros_cmd_vel'], dict)
        self.assertIn('linear', response['ros_cmd_vel'])
        self.assertIsInstance(response['ros_cmd_vel']['linear'], dict)
        self.assertIn('x', response['ros_cmd_vel']['linear'])
        self.assertIn('y', response['ros_cmd_vel']['linear'])

    def test_parse_bb_data_msg(self):
        response_msg = {
            'payload': {
                'dataList': {
                    'ros_ropod_cmd_vel/linear/x':
                    ['[1544441409.0861013, -0.0]', '[1544441409.191301, -0.0]',
                     '[1544441409.3861327, -0.0]', '[1544441409.586039, 0.030635764822363853]',
                     '[1544441409.695998, 0.04727638512849808]',
                     '[1544441409.8860662, 0.18040134012699127]']
                    },
                'receiverId': '347b6674-c915-4940-831e-9e86a0c194f7'
                },
            'header': {
                'metamodel': 'ropod-msg-schema.json',
                'type': 'DATA-QUERY',
                'timestamp': 1561631692.9753046,
                'msgId': 'b62c41fa-f98f-4fb9-9957-51f59819f900'
                }
            }
        variables, data = DataUtils.parse_bb_data_msg(response_msg)
        self.assertIsInstance(variables, list)
        self.assertIsInstance(data, list)
        self.assertEqual(len(variables), len(data))
        for variable_data in data:
            for entry in variable_data:
                self.assertEqual(len(entry), 2)

    def test_parse_bb_latest_data_msg(self):
        response_msg = {
            'payload': {
                'receiverId': '7276ff82-b265-4176-a177-b7936c3c1341',
                'dataList': {'ros_ropod_cmd_vel/linear/x': '[1544441432.830365, 0.0]'}
                },
            'header': {
                'type': 'LATEST-DATA-QUERY',
                'metamodel': 'ropod-msg-schema.json',
                'msgId': 'e46325a0-a57d-4ec9-919b-a32c91f342af',
                'timestamp': 1561636142.9828317}
            }
        variables, data = DataUtils.parse_bb_latest_data_msg(response_msg)
        self.assertIsInstance(variables, list)
        self.assertIsInstance(data, list)
        self.assertEqual(len(variables), len(data))
        for entry in data:
            self.assertEqual(len(entry), 2)

    def test_get_windowed_correlation(self):
        var_pairs, correlation = DataUtils.get_windowed_correlations(
            variable_names=["var1", "var2", "var3"],
            data=np.array([[1, 2, 3], [4, 6, 8], [3, 6, 6]]),
            window_size=2)
        self.assertListEqual([("var1", "var2"), ("var1", "var3"), ("var2", "var3")], var_pairs)
        self.assertTrue(np.array_equal(np.array([[1.0, 1.0], [1.0, 0.0], [1.0, 0.0]]), correlation))

        var_pairs, correlation = DataUtils.get_windowed_correlations(
            variable_names=["var1", "var2", "var3"],
            data=np.array([[1, 2, 3], [4, 8, 8], [3, 6, 6]]),
            window_size=2)
        self.assertListEqual([("var1", "var2"), ("var1", "var3"), ("var2", "var3")], var_pairs)
        self.assertTrue(np.array_equal(np.array([[1.0, 0.0], [1.0, 0.0], [1.0, 1.0]]), correlation))

    def test_find_correlated_variables(self):
        var_pairs = DataUtils.find_correlated_variables(
            variable_names=["var1", "var2", "var3"],
            measurement_matrix=np.array([[1, 2, 3], [4, 6, 8], [3, 6, 7]]))
        self.assertListEqual([("var1", "var2"), ("var1", "var3"), ("var2", "var3")], var_pairs)

    @classmethod
    def _restore_test_db(cls):
        (host, port) = cls._get_db_host_and_port()
        commands = ['mongorestore', cls.test_db_dir, '--db', cls.test_db_name,
                    '--host', host, '--port', str(port)]
        with open(os.devnull, 'w') as devnull:
            process = subprocess.run(commands, stdout=devnull, stderr=devnull)
        return process.returncode == 0

    @classmethod
    def _drop_test_db(cls):
        if cls.test_db_name in cls.client.list_database_names():
            cls.client.drop_database(cls.test_db_name)

    @classmethod
    def _get_db_host_and_port(cls):
        host = 'localhost'
        port = 27017
        if 'DB_HOST' in os.environ:
            host = os.environ['DB_HOST']
        if 'DB_PORT' in os.environ:
            port = int(os.environ['DB_PORT'])
        return (host, port)

if __name__ == '__main__':
    unittest.main()
