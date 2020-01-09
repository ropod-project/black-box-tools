#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from black_box_tools.plot_utils import PlotUtils
from black_box_tools.db_utils import DBUtils
from black_box_tools.data_utils import DataUtils
import black_box_tools.transformations as tf

DB_NAME = 'test_logs'

def get_pos_vel_data(db_name):
    docs = DBUtils.get_docs(db_name, 'ros_amcl_pose')
    num_of_docs = len(docs)
    data = np.zeros((num_of_docs, 5))
    # data (nx6) is arranged as follows
    for i, doc in enumerate(docs):
        if i < 350 or i > 400:
            continue
        # get position information from amcl_pose localisation
        data[i][0] = DataUtils.get_var_value(doc, 'pose/pose/position/x')
        data[i][1] = DataUtils.get_var_value(doc, 'pose/pose/position/y')
        quat_x = DataUtils.get_var_value(doc, 'pose/pose/orientation/x')
        quat_y = DataUtils.get_var_value(doc, 'pose/pose/orientation/y')
        quat_z = DataUtils.get_var_value(doc, 'pose/pose/orientation/z')
        quat_w = DataUtils.get_var_value(doc, 'pose/pose/orientation/w')
        theta = tf.euler_from_quaternion((quat_w, quat_x, quat_y, quat_z))[2]
        data[i][2] = theta
        timestamp = DataUtils.get_var_value(doc, 'timestamp')

        # get velocity information from odom at the same timestamp as position
        odom_doc = DBUtils.get_last_doc_before(db_name, 'ros_ropod_odom', timestamp)
        vel_x = DataUtils.get_var_value(odom_doc, 'twist/twist/linear/x')
        vel_y = DataUtils.get_var_value(odom_doc, 'twist/twist/linear/y')
        data[i][3] = vel_x
        data[i][4] = vel_y

    return data


if __name__ == "__main__":
    DATA = get_pos_vel_data(DB_NAME)
    AXIS = PlotUtils.plot_position_velocity(DATA)
    plt.axis('equal')
    plt.show()
