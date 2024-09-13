#include "robot_localization/navsat_transform.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navsat_transform_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");


  RobotLocalization::NavSatTransform trans(nh, nh_priv);
  ros::spin();

  return EXIT_SUCCESS;
}


