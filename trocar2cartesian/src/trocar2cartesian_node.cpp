#include <iostream>

#include "Trocar2Cartesian.hpp"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "trocar2cartesian");

  Trocar2Cartesian trocar2Cartesian;

  std::cout << "Spinning" << std::endl;
  ros::spin();

  return 0;
}
