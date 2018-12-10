#!/usr/bin/env python

import rospy
import numpy as np
from scipy.interpolate import interp1d

import matplotlib.pyplot as plt
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import sensor_msgs

data_sample = np.array([(-15.4, 4), (-15.3, 5), (-15, 7), (-14, 7.5), (-11, 8.3),
                        (-10, 8.2), (-8, 7.7), (-4, 3.5), (-2.5, 2), (-1, 1), (1, 0.8), (2, 0.8), (4, 1), (7, 2),
                        (11, 6), (14, 9),
                        (15, 11)])


def on_receive_pcl(pcl_data):
    xdata = []
    ydata = []
    for point in pc2.read_points(pcl_data, skip_nans=True):
        pt_x = point[0]
        xdata.append(pt_x)
        pt_y = point[1]
        ydata.append(pt_y)
        # pt_z = point[2]
        #rospy.loginfo("x: %s, y: %s", pt_x, pt_y)
    find_outlier(xdata, ydata, should_plot = False)


def find_outlier(xdata, ydata, limit = 0.65, should_plot = False):
    # data = np.array(
    #     [(-15.4, 4), (-15.3, 5), (-15.2, 6), (-15, 7), (-14, 7.5), (-13, 6.5), (-12.5, 6), (-12, 8), (-11, 8.3),
    #      (-10, 8.2), (-8, 7.7), (-6, 6), (-5, 4.5), (-4, 1), (-4, 3.5), (-2.5, 2), (-1.5, 2), (-1, 1), (0, 1),
    #      (1, 0.8), (2, 0.8), (3, 1), (4, 1), (5, 0), (6, 2), (6, 0), (7, 2), (8, 3), (9, 4), (10, 7), (10, 4.7),
    #      (11, 11.3),
    #      (11, 6), (12, 7), (13, 11.2), (13, 8), (14, 9), (14.5, 10), (15, 11)])

    x = data_sample[:, 0]
    y = data_sample[:, 1]

    #use cubic-spline interpolation
    f2 = interp1d(x, y, kind='cubic')

    outlier_y = []
    outlier_x = []
    i = 0
    int_y = f2(xdata)
    print "Outlier data with +- {0:.2f} limit".format(limit)
    for each_x in xdata:
        # print("x= ", each_x, "y=", ydata[i], "int_y=", int_y[i])
        if math.fabs(int_y[i] - ydata[i]) > limit:
            outlier_y.append(ydata[i])
            outlier_x.append(each_x)
            print "x= {0:.2f}, y= {0:.2f}".format(each_x, ydata[i])
        i += 1
    print "Total {0:d} points outlier".format( len(outlier_y))
    if should_plot:
        print "Plotting..."
        plt.figure()
        plt.plot(xdata, ydata, 'go', x, f2(x), 'b', outlier_x, outlier_y, 'ro')        
        plt.legend(['Data sample', 'Interpolated cubic-spline', 'Outliers'], loc='best')
        plt.axis([min(x) - 1, max(x) + 1, min(y) - 1, max(y) + 1])
        plt.title('Plot outlier')
        plt.show()


if __name__ == '__main__':
    rospy.init_node('pcl_filter_node', log_level=rospy.DEBUG)
    sub = rospy.Subscriber('pcl_pub_path', PointCloud2, callback=on_receive_pcl, queue_size=1)
    rospy.spin()
