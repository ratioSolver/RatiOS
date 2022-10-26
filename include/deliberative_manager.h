#pragma once
#include "deliberative_executor.h"
#include "aerials/ReasonerBuilder.h"
#include "aerials/ReasonerDestroyer.h"
#include "aerials/Executor.h"
#include "aerials/RequirementManager.h"
#include "aerials/TaskExecutor.h"
#include "aerials/TaskLengthener.h"
#include "aerials/TaskCloser.h"
#include "deliberative_tier/Graph.h"
#include "deliberative_tier/Timelines.h"
#include <ros/ros.h>

namespace ratio::ros
{
  class deliberative_executor;

  class deliberative_manager
  {
    friend class deliberative_executor;

  public:
    deliberative_manager(ros::NodeHandle &h);

    ros::NodeHandle &get_handle() { return handle; }

  private:
    void tick();

    void create_reasoner(aerials::ReasonerBuilder::Request &req, aerials::ReasonerBuilder::Response &res);
    void executor(aerials::Executor::Request &req, aerials::Executor::Response &res);
    void destroy_reasoner(aerials::ReasonerDestroyer::Request &req, aerials::ReasonerDestroyer::Response &res);
    void new_requirements(aerials::RequirementManager::Request &req, aerials::RequirementManager::Response &res);
    void lengthen_task(aerials::TaskLengthener::Request &req, aerials::TaskLengthener::Response &res);
    void close_task(aerials::TaskCloser::Request &req, aerials::TaskCloser::Response &res);

  private:
    ros::NodeHandle &handle;
    ros::Timer timer;
    ros::ServiceServer create_reasoner_server;
    ros::ServiceServer start_execution_server;
    ros::ServiceServer reasoner_destroyer_server;
    ros::ServiceServer requirement_manager_server;
    ros::ServiceServer task_lengthener_server;
    ros::ServiceServer task_closer_server;
    ros::Publisher state_publisher;
    ros::Publisher graph_publisher;
    ros::Publisher timelines_publisher;
    ros::ServiceClient can_start;
    ros::ServiceClient start_task;
    ros::ServiceClient can_end;
    ros::ServiceClient end_task;
    std::unordered_map<uint64_t, std::unique_ptr<deliberative_executor>> executors;
  };
} // namespace ratio::ros
