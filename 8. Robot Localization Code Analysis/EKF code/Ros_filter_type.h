#ifndef ROBOT_LOCALIZATION_ROS_FILTER_TYPES_H
#define ROBOT_LOCALIZATION_ROS_FILTER_TYPES_H

#include "robot_localization/ros_filter.h"
#include "robot_localization/ekf.h"
#include "robot_localization/ukf.h"

namespace RobotLocalization
{

//RosFilter의 EKF를 ROSEkf로 이름 재정의
typedef RosFilter<Ekf> RosEkf;
typedef RosFilter<Ukf> RosUkf;

}

#endif  // ROBOT_LOCALIZATION_ROS_FILTER_TYPES_H