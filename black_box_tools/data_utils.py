import numpy as np

class DataUtils(object):
    @staticmethod
    def get_all_measurements(data_dicts_list, var_name, number_of_item_instances=-1):
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

        '''
        number_of_docs = len(data_dicts_list)
        data = np.empty_like(number_of_docs)
        if number_of_item_instances != -1:
            data = np.zeros((number_of_docs, number_of_item_instances))
            for i in range(number_of_item_instances):
                item_var_name = var_name.replace('*', str(i+1))
                item_data = np.array([DataUtils.get_var_value(doc, item_var_name)
                                      for doc in data_dicts_list])
                data[:, i] = item_data
        else:
            data = np.array([DataUtils.get_var_value(doc, var_name) for doc in data_dicts_list])
        return data

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
