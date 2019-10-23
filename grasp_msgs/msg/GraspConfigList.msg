# This message stores a list of grasp configurations.

# The time of acquisition, and the coordinate frame ID.
std_msgs/Header header

# The list of grasp configurations.
grasp_msgs/GraspConfig[] grasps

# Name of the known object these grasps associated to.
string object_name
