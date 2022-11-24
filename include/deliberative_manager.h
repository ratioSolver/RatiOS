#pragma once
#include "deliberative_executor.h"
#include "rclcpp/rclcpp.hpp"
#include "aerials/srv/reasoner_builder.hpp"
#include "aerials/srv/reasoner_destroyer.hpp"
#include "aerials/srv/executor.hpp"
#include "aerials/srv/requirement_manager.hpp"
#include "aerials/srv/task_executor.hpp"
#include "aerials/srv/task_delayer.hpp"
#include "aerials/srv/task_closer.hpp"
#include "deliberative_tier/msg/graph.hpp"
#include "deliberative_tier/msg/timelines.hpp"

namespace ratio::ros
{
  class deliberative_executor;

  class deliberative_manager
  {
    friend class deliberative_executor;

  public:
    deliberative_manager();

    rclcpp::Node::SharedPtr get_node() { return node; }

  private:
    void tick();

    void create_reasoner(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::ReasonerBuilder::Request> req, std::shared_ptr<aerials::srv::ReasonerBuilder::Response> res);
    void executor(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::Executor::Request> req, std::shared_ptr<aerials::srv::Executor::Response> res);
    void destroy_reasoner(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::ReasonerDestroyer::Request> req, std::shared_ptr<aerials::srv::ReasonerDestroyer::Response> res);
    void new_requirements(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::RequirementManager::Request> req, std::shared_ptr<aerials::srv::RequirementManager::Response> res);
    void delay_task(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::TaskDelayer::Request> req, std::shared_ptr<aerials::srv::TaskDelayer::Response> res);
    void extend_task(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::TaskDelayer::Request> req, std::shared_ptr<aerials::srv::TaskDelayer::Response> res);
    void close_task(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::TaskCloser::Request> req, std::shared_ptr<aerials::srv::TaskCloser::Response> res);

  private:
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("deliberative_manager");
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<aerials::srv::ReasonerBuilder>::SharedPtr create_reasoner_server = node->create_service<aerials::srv::ReasonerBuilder>("reasoner_builder", std::bind(&deliberative_manager::create_reasoner, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    rclcpp::Service<aerials::srv::Executor>::SharedPtr start_execution_server = node->create_service<aerials::srv::Executor>("executor", std::bind(&deliberative_manager::executor, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    rclcpp::Service<aerials::srv::ReasonerDestroyer>::SharedPtr reasoner_destroyer_server = node->create_service<aerials::srv::ReasonerDestroyer>("reasoner_destroyer", std::bind(&deliberative_manager::destroy_reasoner, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    rclcpp::Service<aerials::srv::RequirementManager>::SharedPtr requirement_manager_server = node->create_service<aerials::srv::RequirementManager>("requirement_manager", std::bind(&deliberative_manager::new_requirements, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    rclcpp::Service<aerials::srv::TaskDelayer>::SharedPtr task_delayer_server = node->create_service<aerials::srv::TaskDelayer>("task_delayer", std::bind(&deliberative_manager::delay_task, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    rclcpp::Service<aerials::srv::TaskDelayer>::SharedPtr task_extender_server = node->create_service<aerials::srv::TaskDelayer>("task_extender", std::bind(&deliberative_manager::extend_task, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    rclcpp::Service<aerials::srv::TaskCloser>::SharedPtr task_closer_server = node->create_service<aerials::srv::TaskCloser>("task_closer", std::bind(&deliberative_manager::close_task, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    rclcpp::Publisher<aerials::msg::DeliberativeState>::SharedPtr state_publisher = node->create_publisher<aerials::msg::DeliberativeState>("deliberative_state", rclcpp::QoS(10).transient_local());
    rclcpp::Publisher<deliberative_tier::msg::Graph>::SharedPtr graph_publisher = node->create_publisher<deliberative_tier::msg::Graph>("graph", rclcpp::QoS(10).transient_local());
    rclcpp::Publisher<deliberative_tier::msg::Timelines>::SharedPtr timelines_publisher = node->create_publisher<deliberative_tier::msg::Timelines>("timelines", rclcpp::QoS(10).transient_local());
    rclcpp::CallbackGroup::SharedPtr callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::Client<aerials::srv::TaskExecutor>::SharedPtr can_start = node->create_client<aerials::srv::TaskExecutor>("can_start", ::rmw_qos_profile_default, callback_group);
    rclcpp::Client<aerials::srv::TaskExecutor>::SharedPtr start_task = node->create_client<aerials::srv::TaskExecutor>("start_task", ::rmw_qos_profile_default, callback_group);
    rclcpp::Client<aerials::srv::TaskExecutor>::SharedPtr can_end = node->create_client<aerials::srv::TaskExecutor>("can_end", ::rmw_qos_profile_default, callback_group);
    rclcpp::Client<aerials::srv::TaskExecutor>::SharedPtr end_task = node->create_client<aerials::srv::TaskExecutor>("end_task", ::rmw_qos_profile_default, callback_group);
    std::unordered_map<uint64_t, std::unique_ptr<deliberative_executor>> executors;
  };
} // namespace ratio::ros
