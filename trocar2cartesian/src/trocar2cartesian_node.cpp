#include <iostream>

#include "Trocar2Cartesian.hpp"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "trocar2cartesian");
  ros::NodeHandle nh;

  std::string baseTfName;
  nh.param<std::string>("base_tf_name", baseTfName, "robot_base");
  std::string flangeTfName;
  nh.param<std::string>("flange_tf_name", flangeTfName, "robot_flange");

  Trocar2Cartesian trocar2Cartesian(baseTfName, flangeTfName);

  std::cout << "Spinning" << std::endl;
  ros::spin();

  return 0;
}
