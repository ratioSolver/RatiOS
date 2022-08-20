#pragma once
#include "rclcpp/rclcpp.hpp"

namespace ratio::ros
{
  class deliberative_manager : public rclcpp::Node
  {
  public:
    deliberative_manager();
    ~deliberative_manager();

  private:
    void tick();

  private:
    rclcpp::TimerBase::SharedPtr timer;
  };
} // namespace ratio::ros
