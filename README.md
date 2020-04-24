[![Build Status](https://travis-ci.org/ropod-project/black-box-tools.svg?branch=master)](https://travis-ci.org/ropod-project/black-box-tools)

# Summary

A collection of tools for offline processing of data from a robotic black box (as implemented in https://github.com/ropod-project/black-box).

The toolbox primarily uses Jupyter notebooks for data plotting and analysis with the hope that this simplifies the sharing, presentation, and above all, processing of data.

# Dependencies

* Data processing relies on the `black_box_utils` package that is included here as a standalone Python library
* `pymongo`
* `numpy`
* `matplotlib`
* `scipy`
* `rospy`
* `rospy_message_converter`
* `yaml`
* `termcolor`

# notebooks

The following notebooks are provided with this toolbox:
* `sw_data_plot`: Plots data from the smart wheels of a platform
* `cmd_vel_plot`: Plots velocity commands
* `cmd_vel_plot-multi_db`:
* `pivot_encoder_plot-multi_db`:

# sw_data_plot

Plots various smart wheel measurements, in particular:
* wheel currents
* wheel voltage measurements
* wheel velocities
* IMU acceleration measurements per wheel
* IMU gyroscope measurements per wheel

# cmd_vel_plot

Plots planar base velocity commands, namely:
* linear velocity along x
* linear velocity along y
* angular velocity

# black_box_utils

A Python package implementing various utilities for working with black box data. There are three scripts that are part of this package: `data_utils`, `db_utils`, and `plot_utils`.

## db_utils

Defines a `DBUtils` class with the following static methods:

* `restore_db`: Restores a MongoDB database dumped in a given directory
* `get_all_docs`: Returns all documents contained in a specified collection of a given database
* `restore_subdbs`: Restores multiple dumps into a single database.
* `dump_db`: Dumps a MongoDB database in the specified directory.
* `get_data_collection_names`: Returns the names of all black box data collections in the specified database.
* `clear_db`: Drop all collections in the given database.
* `drop_db`: Drops the given database.
* `get_subdb_metadata`: Returns a list of dictionaries containing metadata about any sub-databases stored in the given database.
* `get_subdb_docs`: Returns a list of dictionaries in which the keys are sub-database names and the values are lists of documents in the given collection corresponding to the respective sub-databases.
* `get_docs`: Returns all documents contained in the specific collection of the given database within given time duration.
* `get_docs_of_last_n_secs`: Return documents from given collection name of last n seconds from given db name.
* `get_doc_cursor`: Returns a cursor for all documents in the specified collection of the given database which have the given 'timestamp' value in the given range.
* `get_collection_metadata`: Returns the entry of the 'black_box_metadata' collection for the specified collection.
* `get_oldest_doc`: Returns the oldest document in the given collection name.
* `get_newest_doc`: Returns the newest document in the given collection name.
* `get_last_doc_before`: Returns the last document in the collection with the given name that is before given timestamp.
* `get_db_oldest_timestamp`: Gets the oldest record in the mongo db and returns the corresponding timestamp.
* `get_db_newest_timestamp`: Gets the newest record in the mongo db and returns the corresponding timestamp.
* `get_db_client`: Returns a MongoDB client at <host>:<port>.
* `get_db_host_and_port`: Returns a (host, port) tuple which is ("localhost", 27017) by default, but the values can be overridden by setting the environment variables "DB_HOST" and "DB_PORT" respectively.

## data_utils

Defines a `DataUtils` class with the following static methods:

* `get_all_measurements`: Returns all measurements of a single variables (that potentially has multiple instances - e.g. one instance per wheel)
* `filter_data`: Filters data using a given data filter and returns the filtered array.
* `find_correlated_variables`: Returns a list of variable name pairs where each pair denotes that two measurements in a given measurement matrix are correlated (based on the Pearson correlation coefficient). 
* `get_windowed_correlations`: Returns a list of variable name pairs and a 2D list of windowed pairwise correlations, given a list of variable names and a data array.
* `get_var_value`: Returns the value of a given variable in the given dictionary;
* `get_variable_list`: Returns a list of the names of all variables stored in a given collection.
* `get_flattened_variable_names`: Recursive method that returns a flattened list of the variable names in a given dictionary
* `expand_var_names`: Generates a new list of variable names from a given list of such that the * character in each entry of the original list is replaced by a zero-based index
* `get_bb_query_msg_template`: 'Returns a dictionary which represents a template for a black box query message.
* `get_bb_query_msg`: Returns a black box data query message.
* `get_bb_latest_data_query_msg`: Returns a black box latest data query message.
* `parse_bb_variable_msg`: Returns a nested dictionary that reconstructs the structure of the data represented by the variables in the list of a given black box variable message dictionary.
* `parse_bb_data_msg`: Returns a tuple (variables, data), where variables is a list of variables that were queried and data a list of variable values.
* `parse_bb_latest_data_msg`: Returns a tuple (variables, data), where variables is a list of variables that were queried and data a list of the latest variable values.
* `split_into_windows`: Given a one-dimensional list of elements "data", returns a 2D numpy array of sliding windows of size "window_size".
* `safe_literal_eval`: Uses ast.literal_eval to parse a given string.

## plot_utils

Defines a `PlotUtils` class with the following static methods:
* `subplot_data`: Plots a single time series on a subplot (and potentially annotates any events of interest)
* `subplot_data_lists`: Plots multiple time series on a single subplot (and potentially annotates any events of interest)
* `plot_position_velocity`: Plots the position of robot with dots and its velocity at each position with a line. Colors represent the intensity of velocity.
