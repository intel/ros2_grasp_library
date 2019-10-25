#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interface/control_ur.hpp>

/* pose in joint values*/
static const std::vector<double> HOME = {0.87, -1.44, 1.68, -1.81, -1.56, 0};
/* pose in [x, y, z, R, P, Y]*/
static const std::vector<double> CORNER1_POSE = { 0.1, -0.65, 0.15, 3.14, 0, -3.14};
static const std::vector<double> CORNER2_POSE = {-0.1, -0.45, 0.15, 3.14, 0, -3.14};
static const std::vector<double> CORNER3_POSE = {-0.1, -0.65, 0.15, 3.14, 0, -3.14};
static const std::vector<double> CORNER4_POSE = { 0.1, -0.45, 0.15, 3.14, 0, -3.14};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // init robot control
  auto robot = std::make_shared<URControl>("robot_control",
       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot->parseArgs();
  robot->startLoop();
  rclcpp::sleep_for(1s);

  // Move to home
  robot->moveToJointValues(HOME, 1.05, 1.4);

  // Move to the first corner
  robot->moveToTcpPose(CORNER1_POSE[0], CORNER1_POSE[1], CORNER1_POSE[2], 
                       CORNER1_POSE[3], CORNER1_POSE[4], CORNER1_POSE[5], 1.05, 1.4);

  robot->moveToTcpPose(CORNER1_POSE[0], CORNER1_POSE[1], CORNER1_POSE[2] - 0.05, 
                       CORNER1_POSE[3], CORNER1_POSE[4], CORNER1_POSE[5], 1.05, 1.4);

  // Move to the second corner
  robot->moveToTcpPose(CORNER2_POSE[0], CORNER2_POSE[1], CORNER2_POSE[2], 
                       CORNER2_POSE[3], CORNER2_POSE[4], CORNER2_POSE[5] - 0.05, 1.05, 1.4);

  robot->moveToTcpPose(CORNER2_POSE[0], CORNER2_POSE[1], CORNER2_POSE[2], 
                       CORNER2_POSE[3], CORNER2_POSE[4], CORNER2_POSE[5], 1.05, 1.4);

  // Move to the third corner
  robot->moveToTcpPose(CORNER3_POSE[0], CORNER3_POSE[1], CORNER3_POSE[2],
                       CORNER3_POSE[3], CORNER3_POSE[4], CORNER3_POSE[5], 1.05, 1.4);

  robot->moveToTcpPose(CORNER3_POSE[0], CORNER3_POSE[1], CORNER3_POSE[2] - 0.05, 
                       CORNER3_POSE[3], CORNER3_POSE[4], CORNER3_POSE[5], 1.05, 1.4);

  // Move to the fourth corner
  robot->moveToTcpPose(CORNER4_POSE[0], CORNER4_POSE[1], CORNER4_POSE[2] - 0.05,
                       CORNER4_POSE[3], CORNER4_POSE[4], CORNER4_POSE[5] - 0.05, 1.05, 1.4);

  robot->moveToTcpPose(CORNER4_POSE[0], CORNER4_POSE[1], CORNER4_POSE[2],
                       CORNER4_POSE[3], CORNER4_POSE[4], CORNER4_POSE[5], 1.05, 1.4);

  // Move back to home
  robot->moveToJointValues(HOME, 1.05, 1.4);

  rclcpp::shutdown();
  return 0;

}
