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
                                           case it is ignored)

        '''
        number_of_docs = len(data_dicts_list)
        data = np.empty_like(number_of_docs)
        if number_of_item_instances != -1:
            data = np.zeros((number_of_docs, number_of_item_instances))
            for i in range(number_of_item_instances):
                item_var_name = var_name.replace('*', str(i+1))
                item_data = np.array([doc[item_var_name] for doc in data_dicts_list])
                data[:, i] = item_data
        else:
            data = np.array([doc[var_name] for doc in data_dicts_list])
        return data
