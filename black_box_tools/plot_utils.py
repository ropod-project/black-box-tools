import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class PlotUtils(object):
    @staticmethod
    def subplot_data(fig: matplotlib.figure.Figure,
                     subplot_params: tuple,
                     timestamps: list, data: np.array,
                     x_label: str, y_label: str,
                     event_timestamps: list=None,
                     data_label: str=None,
                     fontsize: int=30,
                     event_annotation_color='g') -> None:
        '''Plots a given time series data on a subplot.

        Keyword arguments:
        fig -- matplotlib figure handle
        subplot_params -- a tuple with three subplot parameters: number of rows,
                          number of columns, and the number of the current subplot
        timestamps -- a list of timestamps (plotted over the x-axis)
        data -- a one-dimensional numpy array of data items (plotted over the y-axis)
        x_label -- x axis label
        y_label -- y axis label
        event_timestamps -- optional list of timestamps for data
                            annotation; the annotations are plotted as
                            lines at the given timestamps
                            (default None, in which case there are no annotations)
        data_label -- optional label for the plotted data; if a label is given,
                      a legend is added to the plot (default None)
        fontsize -- fontsize for the x and y axes labels (default 30)
        event_annotation_color -- color of event annotation lines (default 'g')

        '''
        fig.add_subplot(*subplot_params)

        # we plot the data
        if data_label:
            plt.plot(timestamps, data, label=data_label)
        else:
            plt.plot(timestamps, data)

        # we add event annotations to the plot if there are any events
        if not event_timestamps is None:
            plt.plot([event_timestamps, event_timestamps],
                     [np.min(data), np.max(data)], event_annotation_color)

        plt.xlabel(x_label, fontsize=fontsize)
        plt.ylabel(y_label, fontsize=fontsize)

        # we add a legend only if a data label is assigned
        if data_label:
            plt.legend()

    @staticmethod
    def subplot_data_lists(fig: matplotlib.figure.Figure,
                           subplot_params: tuple,
                           timestamps: list, data: np.array,
                           x_label: str, y_label: str,
                           event_timestamps: list=None,
                           data_labels: list=None,
                           fontsize: int=30,
                           event_annotation_color='g') -> None:
        '''Plots multiple time series on a single subplot.

        Keyword arguments:
        fig -- matplotlib figure handle
        subplot_params -- a tuple with three subplot parameters: number of rows,
                          number of columns, and the number of the current subplot
        timestamps -- a list of timestamps (plotted over the x-axis)
        data -- a 2D numpy array of time series (plotted over the y-axis), where
                each column represents a single time series
        x_label -- x axis label
        y_label -- y axis label
        event_timestamps -- optional list of timestamps for data
                            annotation; the annotations are plotted as
                            green lines at the given timestamps
                            (default None, in which case there are no annotations)
        data_labels -- a list of optional labels for the plotted data, where the size
                       of the list should match the number of columns in "data";
                       if this parameter is passed, a legend is added to the plot
                       (default None)
        fontsize -- fontsize for the x and y axes labels (default 30)
        event_annotation_color -- color of event annotation lines (default 'g')

        '''
        if data_labels:
            try:
                assert data.shape[1] == len(data_labels)
            except AssertionError:
                print('The length of data_labels should match the number of columns in data')
                return

        fig.add_subplot(*subplot_params)

        # we plot the data
        if data_labels:
            for i in range(data.shape[1]):
                plt.plot(timestamps, data[:, i], label=data_labels[i])
        else:
            for i in range(data.shape[1]):
                plt.plot(timestamps, data[:, i])

        # we add event annotations to the plot if there are any events
        if not event_timestamps is None:
            plt.plot([event_timestamps, event_timestamps],
                     [np.min(data), np.max(data)], event_annotation_color)

        plt.xlabel(x_label, fontsize=fontsize)
        plt.ylabel(y_label, fontsize=fontsize)

        # we add a legend only if a data label is assigned
        if data_labels:
            plt.legend()
