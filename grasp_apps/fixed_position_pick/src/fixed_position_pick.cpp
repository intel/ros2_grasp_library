#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interface/control_ur.hpp>

/* pose in joint values*/
static const std::vector<double> HOME = {0.87, -1.44, 1.68, -1.81, -1.56, 0};
/* pose in [x, y, z, qx, qy, qz, qw]*/
static const std::vector<double> PICK_POSE = { -0.157402, -0.679509, 0.094437, 0.190600, 0.948295, 0.239947, 0.082662};
static const std::vector<double> PLACE_POSE = {-0.350, -0.296, 0.145, -0.311507, 0.950216, -0.004305, 0.005879};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // init robot control
  auto robot = std::make_shared<URControl>("robot_control",
       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot->parseArgs();
  robot->startLoop();
  rclcpp::sleep_for(2s);

  // Move to home
  robot->moveToJointValues(HOME, 1.05, 1.4);

  // Pick
  geometry_msgs::msg::PoseStamped pose_pick;
  pose_pick.header.frame_id = "base";
  pose_pick.header.stamp = robot->now();
  pose_pick.pose.position.x = PICK_POSE[0];
  pose_pick.pose.position.y = PICK_POSE[1];
  pose_pick.pose.position.z = PICK_POSE[2];
  pose_pick.pose.orientation.x = PICK_POSE[3];
  pose_pick.pose.orientation.y = PICK_POSE[4];
  pose_pick.pose.orientation.z = PICK_POSE[5];
  pose_pick.pose.orientation.w = PICK_POSE[6];

  robot->pick(pose_pick, 1.05, 1.4, 0.5, 0.1);

  // Place
  geometry_msgs::msg::PoseStamped pose_place;
  pose_place.header.frame_id = "base";
  pose_place.header.stamp = robot->now();
  pose_place.pose.position.x = PLACE_POSE[0];
  pose_place.pose.position.y = PLACE_POSE[1];
  pose_place.pose.position.z = PLACE_POSE[2];
  pose_place.pose.orientation.x = PLACE_POSE[3];
  pose_place.pose.orientation.y = PLACE_POSE[4];
  pose_place.pose.orientation.z = PLACE_POSE[5];
  pose_place.pose.orientation.w = PLACE_POSE[6];

  robot->place(pose_place, 1.05, 1.4, 0.5, 0.1);

  // Move back to home
  robot->moveToJointValues(HOME, 1.05, 1.4);

  rclcpp::shutdown();
  return 0;

}