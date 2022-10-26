#include "deliberative_manager.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "deliberative_tier");
  ros::NodeHandle nh;
  ROS_INFO("Starting the Deliberative Tier..");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  ratio::ros1::deliberative_manager dm(nh);

  ros::spin();

  ros::shutdown();

  return 0;
}
