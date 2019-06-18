import time
import ast
import numpy as np
import scipy.signal as signal

class Filters(object):
    MEDIAN = 'median'

class DataUtils(object):
    @staticmethod
    def get_all_measurements(data_dicts_list, var_name, number_of_item_instances=-1,
                             data_filter=None, filter_window_size=3):
        '''If there is only a single instance of a variable, "number_of_item_instances"
        should either not be passed or should have the value -1; in this case,
        the function returns a numpy array of shape (len(data_dicts_list),)
        containing all measurements of the given variable.

        If there are multiple instances of a variable, the function returns
        a numpy array of shape (len(data_dicts_list), number_of_item_instances)
        containing all measurements of the given variable for each existing instance.
        In this case, the variable name should contain a * so that each instance
        of the variable can be read.

        Example: If a robot has four wheels and there are four current measurements
        (one per each wheel), the variable name could be "current_*" and we have
        four instances of the variable; the resulting array would thus have
        four columns, where the i-th column contains all measurements for "current_i"

        Keyword arguments:
        @param data_dicts_list -- a list of dictionaries containing data variables
        @param var_name -- name of the variable to be read
        @param number_of_item_instances -- number of instances of the variable
                                           in each data dictionary (default -1, in which
                                           case the variable is ignored)
        @param data_filter -- filter to apply to the data
                              (default None, in which case the raw data are returned)
        @param filter_window_size -- window size for the filter; only used if
                                     data_filter is not None (default 3)

        '''
        number_of_docs = len(data_dicts_list)
        data = np.empty_like(number_of_docs)
        if number_of_item_instances != -1:
            data = np.zeros((number_of_docs, number_of_item_instances))
            for i in range(number_of_item_instances):
                item_var_name = var_name.replace('*', str(i))
                item_data = np.array([DataUtils.get_var_value(doc, item_var_name)
                                      for doc in data_dicts_list])
                if data_filter:
                    item_data = DataUtils.filter_data(item_data, data_filter, filter_window_size)
                data[:, i] = item_data
        else:
            data = np.array([DataUtils.get_var_value(doc, var_name) for doc in data_dicts_list])
            if data_filter:
                data = DataUtils.filter_data(data, data_filter, filter_window_size)
        return data

    @staticmethod
    def filter_data(data, data_filter=Filters.MEDIAN, window_size=3):
        '''Filters "data" using "data_filter" and returns the filtered array.
        The value of "data_filter" should be one of the values specified
        in the data_utils.Filters enum.

        Note: Only Filters.MEDIAN is currently supported.

        Keyword arguments:
        @param data -- a one-dimensional numpy array of measurements
        @param data_filter -- filter to apply to the data (default 'median')
        @param window_size -- windows size for the filter (default 3)

        '''
        filtered_data = np.array(data)
        if data_filter == Filters.MEDIAN:
            filtered_data = signal.medfilt(data, kernel_size=window_size)
        else:
            print('[filter_data] Unknown filter {0} specified'.format(data_filter))
        return filtered_data

    @staticmethod
    def find_correlated_variables(variable_names, measurement_matrix,
                                  corr_threshold=0.9):
        '''Returns a list of variable name pairs where each pair
        (variable_names[i], variable_names[j]) denotes that measurement_matrix[i]
        and measurement_matrix[j] are correlated. The function assumes that
        each row reprents a variable and the columns represent variable measurements.

        Keyword arguments:
        variable_names: Sequence[str] -- a list of variable names
        measurement_matrix: Sequence[Sequence[float]] -- a numpy matrix of measurements
                                                         in which each row represents a
                                                         single variable
        corr_threshold: float -- correlation coefficient threshold: if the absolute value
                                 of the coefficient is above the threshold, the
                                 measurements are assumed to be correlated (default 0.9)

        '''
        corr_matrix = np.corrcoef(measurement_matrix)
        correlated_variables = []
        for i in range(measurement_matrix.shape[0]):
            for j in range(i+1, measurement_matrix.shape[0]):
                if abs(corr_matrix[i, j]) > corr_threshold:
                    correlated_variables.append((variable_names[i],
                                                 variable_names[j]))
        return correlated_variables

    @staticmethod
    def get_var_value(item_dict, var_name):
        '''Returns the value of "var_name" in the given dictionary; returns None
        if the variable does not exit in the dictionary. "var_name" is expected
        to be a flattened variable name with items at different levels
        separated by a / (the name only contain the names of the items
        that are included in the document and should NOT be prepended
        by the name of the collection).

        @param item_dict -- a MongoDB document
        @param var_name -- name of the variable whose value is queried

        Example:
        If "item_dict" is given as
        {
            linear:
            {
                x: 1,
                y: 2,
                z: 2
            }
        }
        and "var_name" is "linear/x", the returned value will be 1.

        If "item_dict" has a list of items, the items should also be separated by
        a / (as done by self.__get_flattened_variable_names). For instance, if
        "item_dict" is given as
        {
            sensors:
            [
                {
                    velocity1: 0.1,
                    velocity2: 0.2
                },
                {
                    velocity1: 0.0,
                    velocity2: 0.1
                }
            ]
        }
        and we want to get the value of the second "velocity2" item,
        "var_name" should be given as "sensors/1/velocity2".

        '''
        current_item = item_dict
        separator_idx = var_name.find('/')
        current_var_name = var_name
        while separator_idx != -1:
            current_var_name = var_name[0:separator_idx]
            if current_var_name.isdigit():
                current_var_name = int(current_var_name)
            current_item = current_item[current_var_name]
            var_name = var_name[separator_idx+1:]
            separator_idx = var_name.find('/')

        val = None
        try:
            if var_name.isdigit():
                var_name = int(var_name)
            val = current_item[var_name]
        except KeyError:
            print('An unknown variable {0} was requested; returning None'.format(var_name))
        return val

    @staticmethod
    def get_variable_list(collection_name, collection):
        '''Returns a list of the names of all variables stored in the input collection

        Keyword arguments:
        @param collection_name -- name of a MongoDB collection (prepended to the variable names)
        @param collection -- a MongoDB collection

        '''
        doc = collection.find_one({})
        variables = DataUtils.get_flattened_variable_names(doc, collection_name)
        return variables

    @staticmethod
    def get_flattened_variable_names(current_dict, current_var_name):
        '''Recursive method that returns a flattened list of the variable names
        in the given dictionary, where variables at different levels are separated by a /.

        @param current_dict -- current variable dictionary (initially a full MongoDB document;
                               the value gets updated recursively)
        @param current_var_name -- name of the variable at the current level of recursion
                                   (initially set to the name of a collection so that the full
                                   variable names are of the form "collection_name/variable_name")

        Example:
        If "current_dict" is initially given as
        {
            linear:
            {
                x: -1,
                y: -1,
                z: -1
            },
            angular:
            {
                x: -1,
                y: -1,
                z: -1
            }
        }
        and "current_var_name" is initially "ros_cmd_vel", the returned list will contain
        the following variable names:
        ["ros_cmd_vel/linear/x", "ros_cmd_vel/linear/y", "ros_cmd_vel/linear/z",
         "ros_cmd_vel/angular/x", "ros_cmd_vel/angular/y", "ros_cmd_vel/angular/z"]

        If the input dictionary contains a list of items, the items will also
        be separated by a /. This is best illustrated by an example. If "current_dict"
        is given as
        {
            commands:
            [
                {
                    setpoint1: -1,
                    setpoint2: -1
                },
                {
                    setpoint1: -1,
                    setpoint2: -1
                }
            ],
            sensors:
            [
                {
                    velocity1: -1,
                    velocity2: -1
                },
                {
                    velocity1: -1,
                    velocity2: -1
                }
            ]
        }
        and "current_var_name" is "ros_sw_data", the returned list will contain
        the following variable names:
        ["ros_sw_data/commands/0/setpoint1", "ros_sw_data/commands/0/setpoint2",
         "ros_sw_data/commands/1/setpoint1", "ros_sw_data/commands/1/setpoint2",
         "ros_sw_data/sensors/0/velocity1", "ros_sw_data/sensors/0/velocity2",
         "ros_sw_data/sensors/1/velocity1", "ros_sw_data/sensors/1/velocity2"]

        '''
        if not isinstance(current_dict, dict):
            return [current_var_name]

        variables = []
        for name in current_dict:
            if name == '_id' or name == 'timestamp':
                continue

            var_list = []
            if not isinstance(current_dict[name], list):
                new_var_name = '{0}/{1}'.format(current_var_name, name)
                var_list = DataUtils.get_flattened_variable_names(current_dict[name],
                                                                  new_var_name)
                variables.extend(var_list)
            else:
                new_var_name = current_var_name
                for i, list_item in enumerate(current_dict[name]):
                    new_var_name = '{0}/{1}/{2}'.format(current_var_name, name, str(i))
                    var_list = DataUtils.get_flattened_variable_names(list_item, new_var_name)
                    variables.extend(var_list)
        return variables

    @staticmethod
    def expand_var_names(variables, index_count):
        '''Generates a new list of variable names from "variables" such that
        the * character in each entry of "variables" is replaced by a zero-based index
        (the number of indices is determined by "index_count").

        Example:
        If "variables" is the list ["var1/*", "var2/*"], and "index_count" is 3, the
        resulting list will be ["var1/0", "var1/1", "var1/2", "var2/0", "var2/1", "var2/2"].

        Keyword arguments:
        variables -- a list of variable names including a * as an index replacement
        index_count -- number of times each variable should be expanded

        '''
        expanded_vars = [var.replace('*', str(i))
                         for var in variables
                         for i in range(index_count)]
        return expanded_vars

    @staticmethod
    def get_bb_query_msg_template():
        '''Returns a dictionary of the form
        {
            'header':
            {
                'metamodel': 'ropod-msg-schema.json'
                'version': '1.0.0'
            },
            'payload': {}
        }
        which represents a template for a black box query message.

        '''
        query_msg = {}
        query_msg['header'] = {}
        query_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        query_msg['header']['version'] = '1.0.0'
        query_msg['payload'] = {}
        return query_msg

    @staticmethod
    def get_bb_query_msg(sender_id, bb_id, variable_list, start_query_time, end_query_time):
        '''Returns a black box data query message.

        Keyword arguments:
        sender_id -- ID of the user that queries the data (typically a session ID)
        bb_id -- ID of the black box that should be queried (of the form black_box_<xxx>)
        variable_list -- a list of variables whose values are queried
        start_query_time -- UNIX timestamp denoting the start data query time
        end_query_time -- UNIX timestamp denoting the end data query time

        '''
        query_msg = DataUtils.get_bb_query_msg_template()
        query_msg['header']['type'] = 'DATA-QUERY'
        query_msg['header']['timestamp'] = time.time()
        query_msg['payload']['senderId'] = sender_id
        query_msg['payload']['blackBoxId'] = bb_id
        query_msg['payload']['variables'] = variable_list
        query_msg['payload']['startTime'] = start_query_time
        query_msg['payload']['endTime'] = end_query_time
        return query_msg

    @staticmethod
    def get_bb_latest_data_query_msg(sender_id, bb_id, variable_list):
        '''Returns a black box latest data query message.

        Keyword arguments:
        sender_id -- ID of the user that queries the data (typically a session ID)
        bb_id -- ID of the black box that should be queried (of the form black_box_<xxx>)
        variable_list -- a list of variables whose values are queried

        '''
        query_msg = DataUtils.get_bb_query_msg_template()
        query_msg['header']['type'] = 'LATEST-DATA-QUERY'
        query_msg['header']['timestamp'] = time.time()
        query_msg['payload']['senderId'] = sender_id
        query_msg['payload']['blackBoxId'] = bb_id
        query_msg['payload']['variables'] = variable_list
        return query_msg

    @staticmethod
    def parse_bb_variable_msg(bb_variable_msg):
        '''Returns a nested dictionary that reconstructs the structure of the
        data represented by the variables in bb_variable_msg["payload"]["variableList"]

        Example:
        If bb_variable_msg["payload"]["variableList"] is the nested dictionary
        {
            'ros': ['ros_cmd_vel/linear/z', 'ros_cmd_vel/linear/x', 'ros_cmd_vel/angular/x',
                    'ros_cmd_vel/angular/z', 'ros_cmd_vel/angular/y', 'ros_cmd_vel/linear/y',
                    'ros_pose/x', 'ros_pose_y', 'ros_pose_z'],
            'zyre': ['zyre_pose/x', 'zyre_pose_y', 'zyre_pose_z']
        }
        the resulting nested dictionary will be
        {
            'ros_cmd_vel':
            {
                'linear':
                {
                    'x': {}
                    'y': {},
                    'z': {}
                },
                'angular':
                {
                    'x': {}
                    'y': {},
                    'z': {}
                }
            },
            'ros_pose':
            {
                'x': {},
                'y': {},
                'z': {}
            },
            'zyre_pose':
            {
                'x': {},
                'y': {},
                'z': {}
            }
        }

        Keyword arguments:
        bb_variable_msg -- a black box variable query response

        '''
        variables = dict()
        if bb_variable_msg:
            for variable_names in bb_variable_msg['payload']['variableList'].values():
                if variable_names:
                    for full_variable_name in variable_names:
                        slash_indices = [0]
                        current_variable_dict = variables
                        for i, char in enumerate(full_variable_name):
                            if char == '/':
                                slash_indices.append(i+1)
                                name_component = full_variable_name[slash_indices[-2]:
                                                                    slash_indices[-1]-1]
                                if name_component not in current_variable_dict:
                                    current_variable_dict[name_component] = {}
                                current_variable_dict = current_variable_dict[name_component]
                        name_component = full_variable_name[slash_indices[-1]:]
                        current_variable_dict[name_component] = {}
        return variables

    @staticmethod
    def parse_bb_data_msg(bb_data_msg):
        '''Returns a tuple (variables, data), where variables is a list of
        variables that were queried and data a list of variable values, namely
        data[i] is a list of [timestamp, value] lists corresponding to variables[i]

        Keyword arguments:
        bb_data_msg -- a black box data query response

        '''
        variables = []
        data = []
        if bb_data_msg:
            for var_name, var_data in bb_data_msg['payload']['dataList'].items():
                variables.append(var_name)
                variable_data_list = [DataUtils.safe_literal_eval(item) for item in var_data]
                data.append(variable_data_list)
        return (variables, data)

    @staticmethod
    def parse_bb_latest_data_msg(bb_data_msg):
        '''Returns a tuple (variables, data), where variables is a list of
        variables that were queried and data a list of the latest variable values,
        namely data[i] is a list [timestamp, value] corresponding to
        the latest value of variables[i] along with its timestamp

        Keyword arguments:
        bb_data_msg -- a black box latest data query response

        '''
        variables = []
        data = []
        if bb_data_msg:
            for var_name, var_data in bb_data_msg['payload']['dataList'].items():
                variables.append(var_name)
                if var_data:
                    data.append(DataUtils.safe_literal_eval(var_data))
                else:
                    data.append(None)
        return (variables, data)

    @staticmethod
    def safe_literal_eval(data_str):
        '''Uses ast.literal_eval to parse the input item. Returns None
        if literal_eval throws an exception.

        Keyword arguments:
        data_str: str -- Item to be parsed

        '''
        try:
            return ast.literal_eval(data_str)
        except ValueError:
            return None
