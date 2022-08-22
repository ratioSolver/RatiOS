#pragma once
#include "deliberative_executor.h"
#include "rclcpp/rclcpp.hpp"
#include "aerials/srv/reasoner_builder.hpp"
#include "aerials/srv/reasoner_destroyer.hpp"
#include "aerials/srv/executor.hpp"
#include "aerials/msg/deliberative_state.hpp"

namespace ratio::ros
{
  class deliberative_executor;

  class deliberative_manager : public rclcpp::Node
  {
    friend class deliberative_executor;

  public:
    deliberative_manager();

  private:
    void tick();

    void create_reasoner(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::ReasonerBuilder::Request> req, std::shared_ptr<aerials::srv::ReasonerBuilder::Response> res);
    void start_execution(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::Executor::Request> req, std::shared_ptr<aerials::srv::Executor::Response> res);
    void destroy_reasoner(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::ReasonerDestroyer::Request> req, std::shared_ptr<aerials::srv::ReasonerDestroyer::Response> res);

  private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<aerials::srv::ReasonerBuilder>::SharedPtr create_reasoner_server = create_service<aerials::srv::ReasonerBuilder>("reasoner_builder", std::bind(&deliberative_manager::create_reasoner, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    rclcpp::Service<aerials::srv::Executor>::SharedPtr start_execution_server = create_service<aerials::srv::Executor>("start_execution", std::bind(&deliberative_manager::start_execution, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    rclcpp::Service<aerials::srv::ReasonerDestroyer>::SharedPtr destroy_reasoner_server = create_service<aerials::srv::ReasonerDestroyer>("destroy_reasoner", std::bind(&deliberative_manager::destroy_reasoner, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    std::unordered_map<uint64_t, std::unique_ptr<deliberative_executor>> executors;
    rclcpp::Publisher<aerials::msg::DeliberativeState>::SharedPtr state_publisher = create_publisher<aerials::msg::DeliberativeState>("deliberative_state", rclcpp::QoS(10).transient_local());
  };
} // namespace ratio::ros
