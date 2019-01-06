# This message describes a set of point samples at which to detect grasps.

# Header
std_msgs/Header header

# The samples, as (x,y,z) points, at which to search for grasp candidates. 
geometry_msgs/Point[] samples
