# Task #
> Create a ROS node using C++ that reads in the x-y values in the table in Appendix and publish it as pointcloud.
You may copy the coordinates into a separate file.
>>To launch node:
     roslaunch pcl_nodes pcl_publisher.launch

> Add a service server in the ROS node created in 1. When this service is called, it shall return the number of points in the pointcloud.
Request: empty
Response: number of points in the pointcloud
>> rosservice call /get_number_of_points

> Algorithm to find outliers
Objective: 
Given the attached set of (x, y) coordinates in a plane, print a reasonable set of outlier points such that the remaining points form a smooth curve.
You can freely propose any methods and criteria on what forms a reasonable set.
You may use any language, but preferred in python or C++. You can freely use any library to help you to solve the problem.
 >>This is handle by pcl_filter node
    roslaunch pcl_nodes pcl_filter.launch 