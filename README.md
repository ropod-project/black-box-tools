# Summary

A collection of tools for offline processing of data from a robotic black box (as implemented in https://github.com/ropod-project/black-box).

The toolbox primarily uses Jupyter notebooks for data plotting and analysis with the hope that this simplifies the sharing, presentation, and above all processing of data.

# Dependencies

* Data processing relies on the `black_box_utils` package that is included here as a standalone Python library
* `pymongo`
* `numpy`
* `matplotlib`

# notebooks

The following notebooks are provided with this toolbox:
* `sw_data_plot`: Plots data from the smart wheels of a platform
* `cmd_vel_plot`: Plots velocity commands

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


## data_utils

Defines a `DataUtils` class with the following static methods:

* `get_all_measurements`: Returns all measurements of a single variables (that potentially has multiple instances - e.g. one instance per wheel)

## plot_utils

Defines a `PlotUtils` class with the following static methods:
* `subplot_data`: Plots a single time series on a subplot (and potentially annotates any events of interest)
* `subplot_data_lists`: Plots multiple time series on a single subplot (and potentially annotates any events of interest)
