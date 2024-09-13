#include "robot_localization/navsat_transform.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <memory>

namespace RobotLocalization
{

class NavSatTransformNodelet : public nodelet::Nodelet
{
private:
  std::unique_ptr<RobotLocalization::NavSatTransform> trans;

public:
  virtual void onInit()
  {
    NODELET_DEBUG("Initializing nodelet...");

    ros::NodeHandle nh      = getNodeHandle();
    ros::NodeHandle nh_priv = getPrivateNodeHandle();

    trans = std::make_unique<RobotLocalization::NavSatTransform>(nh, nh_priv);
  }
};

}  // namespace RobotLocalization

PLUGINLIB_EXPORT_CLASS(RobotLocalization::NavSatTransformNodelet, nodelet::Nodelet);