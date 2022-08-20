#include "deliberative_manager.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ratio::ros::deliberative_manager>());
  rclcpp::shutdown();
  return 0;
}
