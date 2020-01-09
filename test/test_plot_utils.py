#! /usr/bin/env python3

import matplotlib.pyplot as plt
from black_box_tools.plot_utils import PlotUtils

DB_NAME = 'test_logs'

def main():
    axis = PlotUtils.plot_position_velocity_from_bbdb(DB_NAME)
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    main()
