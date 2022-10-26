#include "deliberative_manager.h"

using namespace std::chrono_literals;

namespace ratio::ros
{
    deliberative_manager::deliberative_manager(ros::NodeHandle &h) : handle(h),
                                                                     timer(h.createTimer(ros::Duration(1.0), std::bind(&deliberative_manager::tick, &dm))),
                                                                     create_reasoner_server(h.advertiseService("reasoner_builder", &deliberative_manager::create_reasoner, this)),
                                                                     start_execution_server(h.advertiseService("executor", &deliberative_manager::executor, this)),
                                                                     reasoner_destroyer_server(h.advertiseService("reasoner_destroyer", &deliberative_manager::destroy_reasoner, this)),
                                                                     requirement_manager_server(h.advertiseService("requirement_manager", &deliberative_manager::new_requirements, this)),
                                                                     task_lengthener_server(h.advertiseService("task_lengthner", &deliberative_manager::lengthen_task, this)),
                                                                     task_closer_server(h.advertiseService("task_closer", &deliberative_manager::close_task, this)),
                                                                     state_publisher(h.advertise<aerials::DeliberativeState>("deliberative_state", 10, true)),
                                                                     graph_publisher(h.advertise<deliberative_tier::Graph>("graph", 10)),
                                                                     timelines_publisher(h.advertise<deliberative_tier::Timelines>("timelines", 10)),
                                                                     can_start(h.serviceClient<aerials::TaskExecutor>("can_start")),
                                                                     start_task(h.serviceClient<aerials::TaskExecutor>("start_task")),
                                                                     can_end(h.serviceClient<aerials::TaskExecutor>("can_end")),
                                                                     end_task(h.serviceClient<aerials::TaskExecutor>("end_task"))

    {
        while (!can_start.waitForExistence(1s) || !start_task.waitForExistence(1s) || !can_end.waitForExistence(1s) || !end_task.waitForExistence(1s))
        {
            if (!ros::ok())
            {
                ROS_ERROR("Interrupted while waiting for the task execution services. Exiting.");
                exit(0);
            }
            ROS_INFO("Task execution services not available, waiting again...");
        }
    }

    void deliberative_manager::tick()
    {
        for (auto &exec : executors)
            exec.second->tick();
    }

    void deliberative_manager::create_reasoner(aerials::ReasonerBuilder::Request &req, aerials::ReasonerBuilder::Response &res)
    {
        res.reasoner_id = executors.size();
        ROS_DEBUG("Creating new reasoner #%lu..", res.reasoner_id);

        executors[res.reasoner_id] = std::make_unique<deliberative_executor>(*this, res.reasoner_id, req.domain_files, req.requirements);
        res.consistent = executors[res.reasoner_id].current_state != aerials::DeliberativeState::INCONSISTENT;
    }

    void deliberative_manager::executor(aerials::Executor::Request &req, aerials::Executor::Response &res)
    {
        ROS_DEBUG("Staarting execution for reasoner #%lu..", req.reasoner_id);
        if (const auto exec = executors.find(req.reasoner_id); exec != executors.end())
        {
            switch (req.command)
            {
            case aerials::srv::Executor::Request::START:
                exec->second->start_execution(req.notify_start, req.notify_end);
                break;
            case aerials::srv::Executor::Request::PAUSE:
                exec->second->pause_execution();
                break;
            case aerials::srv::Executor::Request::STOP:
                exec->second->stop_execution();
                break;
            }
            res.new_state = exec->second->current_state;
        }
        else
        {
            ROS_WARN("Reasoner #%lu does not exists..", req.reasoner_id);
            res.new_state = aerials::DeliberativeState::DESTROYED;
        }
    }

    void deliberative_manager::destroy_reasoner(aerials::ReasonerDestroyer::Request &req, aerials::ReasonerDestroyer::Response &res)
    {
        ROS_DEBUG("Destroying reasoner #%lu..", req.reasoner_id);
        if (const auto exec = executors.find(req.reasoner_id); exec != executors.end())
        {
            executors.erase(exec);
            res.destroyed = true;

            auto state_msg = aerials::DeliberativeState();
            state_msg.reasoner_id = req.reasoner_id;
            state_msg.deliberative_state = aerials::DeliberativeState::DESTROYED;
            state_publisher.publish(state_msg);
        }
        else
        {
            ROS_WARN("Reasoner #%lu does not exists..", req.reasoner_id);
            res.destroyed = false;
        }
    }

    void deliberative_manager::new_requirements(aerials::RequirementManager::Request &req, aerials::RequirementManager::Response &res)
    {
        ROS_DEBUG("Adding new requirements to reasoner #%lu..", req.reasoner_id);
        if (const auto exec = executors.find(req.reasoner_id); exec != executors.end())
        {
            exec->second->append_requirements(req.requirements);
            res.consistent = exec->second->current_state != aerials::DeliberativeState::INCONSISTENT;
        }
        else
        {
            ROS_WARN("Reasoner #%lu does not exist..", req.reasoner_id);
            res.consistent = false;
        }
    }

    void deliberative_manager::lengthen_task(aerials::TaskLengthener::Request &req, aerials::TaskLengthener::Response &res)
    {
        ROS_DEBUG("Stretching task %lu of reasoner #%lu..", req.task.task_id, req.task.reasoner_id);
        if (const auto exec = executors.find(req.task.reasoner_id); exec != executors.end())
        {
            exec->second->lengthen_task(req.task.task_id, semitone::rational(req.delay.num, req.delay.den));
            res.lengthened = true;
        }
        else
        {
            ROS_WARN("Reasoner #%lu does not exist..", req.task.reasoner_id);
            res.lengthened = false;
        }
    }

    void deliberative_manager::close_task(aerials::TaskCloser::Request &req, aerials::TaskCloser::Response &res)
    {
        ROS_DEBUG("Closing task %lu of reasoner #%lu..", req.task.task_id, req.task.reasoner_id);
        if (const auto exec = executors.find(req.task.reasoner_id); exec != executors.end())
        {
            exec->second->close_task(req.task.task_id, req.success);
            res.closed = true;
        }
        else
        {
            ROS_WARN("Reasoner #%lu does not exist..", req.task.reasoner_id);
            res.closed = false;
        }
    }
} // namespace ratio::ros
