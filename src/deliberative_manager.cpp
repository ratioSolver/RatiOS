#include "deliberative_manager.h"

using namespace std::chrono_literals;

namespace ratio::ros
{
    deliberative_manager::deliberative_manager() : timer(node->create_wall_timer(1s, std::bind(&deliberative_manager::tick, this)))
    {
        while (!can_start->wait_for_service(1s) || !start_task->wait_for_service(1s) || !can_end->wait_for_service(1s) || !end_task->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the task execution services. Exiting.");
                exit(0);
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task execution services not available, waiting again...");
        }
    }

    void deliberative_manager::tick()
    {
        for (auto &exec : executors)
            exec.second->tick();
    }

    void deliberative_manager::create_reasoner([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::ReasonerBuilder::Request> req, std::shared_ptr<aerials::srv::ReasonerBuilder::Response> res)
    {
        res->reasoner_id = executors.size();
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Creating new reasoner #%lu..", res->reasoner_id);

        executors[res->reasoner_id] = std::make_unique<deliberative_executor>(*this, res->reasoner_id, req->domain_files, req->requirements);
        res->consistent = executors[res->reasoner_id]->state != aerials::msg::DeliberativeState::INCONSISTENT;
    }

    void deliberative_manager::start_execution([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::Executor::Request> req, std::shared_ptr<aerials::srv::Executor::Response> res)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Staarting execution for reasoner #%lu..", req->reasoner_id);
        if (const auto exec = executors.find(req->reasoner_id); exec != executors.end())
        {
            exec->second->start_execution(req->notify_start, req->notify_end);
            res->new_state = exec->second->state;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Reasoner #%lu does not exists..", req->reasoner_id);
            res->new_state = aerials::msg::DeliberativeState::DESTROYED;
        }
    }

    void deliberative_manager::destroy_reasoner([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::ReasonerDestroyer::Request> req, std::shared_ptr<aerials::srv::ReasonerDestroyer::Response> res)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Destroying reasoner #%lu..", req->reasoner_id);
        if (const auto exec = executors.find(req->reasoner_id); exec != executors.end())
        {
            executors.erase(exec);
            res->destroyed = true;

            auto state_msg = aerials::msg::DeliberativeState();
            state_msg.reasoner_id = req->reasoner_id;
            state_msg.deliberative_state = aerials::msg::DeliberativeState::DESTROYED;
            state_publisher->publish(state_msg);
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Reasoner #%lu does not exists..", req->reasoner_id);
            res->destroyed = false;
        }
    }

    void deliberative_manager::new_requirements([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::RequirementManager::Request> req, std::shared_ptr<aerials::srv::RequirementManager::Response> res)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Adding new requirements to reasoner #%lu..", req->reasoner_id);
        if (const auto exec = executors.find(req->reasoner_id); exec != executors.end())
        {
            exec->second->append_requirements(req->requirements);
            res->consistent = exec->second->state != aerials::msg::DeliberativeState::INCONSISTENT;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Reasoner #%lu does not exist..", req->reasoner_id);
            res->consistent = false;
        }
    }

    void deliberative_manager::close_task([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::TaskCloser::Request> req, std::shared_ptr<aerials::srv::TaskCloser::Response> res)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Closing task %lu of reasoner #%lu..", req->task.task_id, req->task.reasoner_id);
        if (const auto exec = executors.find(req->task.reasoner_id); exec != executors.end())
        {
            exec->second->close_task(req->task.task_id, req->success);
            res->closed = true;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Reasoner #%lu does not exist..", req->task.reasoner_id);
            res->closed = false;
        }
    }
} // namespace ratio::ros
