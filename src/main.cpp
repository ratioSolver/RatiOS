#include "deliberative_manager.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  ratio::ros::deliberative_manager dm;
  rclcpp::spin(dm.get_node());

  rclcpp::shutdown();
  return 0;
}
