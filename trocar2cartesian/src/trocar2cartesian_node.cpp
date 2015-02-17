#include <iostream>

#include "Trocar2Cartesian.hpp"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "trocar2cartesian");
  ros::NodeHandle pnh("~");

  std::string robotName;
  pnh.param<std::string>("robot_name", robotName, "robot");
  std::string baseTfName;
  pnh.param<std::string>("base_tf_name", baseTfName, "robot_base");
  std::string flangeTfName;
  pnh.param<std::string>("flange_tf_name", flangeTfName, "robot_flange");

  Trocar2Cartesian trocar2Cartesian(robotName, baseTfName, flangeTfName);

  std::cout << "Spinning" << std::endl;
  ros::MultiThreadedSpinner spinner(4); // >=2 threads required to handle callbacks in parallel to setTrocarCallback()
  spinner.spin();

  return 0;
}
