#include "deliberative_manager.h"

using namespace std::chrono_literals;

namespace ratio::ros
{
    deliberative_manager::deliberative_manager() : Node("deliberative_manager"), timer(create_wall_timer(1s, std::bind(&deliberative_manager::tick, this))) {}

    void deliberative_manager::tick() {}

    void deliberative_manager::create_reasoner([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::ReasonerBuilder::Request> req, std::shared_ptr<aerials::srv::ReasonerBuilder::Response> res)
    {
        res->reasoner_id = executors.size();
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Creating new reasoner %lu..", res->reasoner_id);

        executors[res->reasoner_id] = std::make_unique<deliberative_executor>(*this, res->reasoner_id, req->domain_files, req->requirements);

        auto state_msg = aerials::msg::DeliberativeState();
        state_msg.reasoner_id = res->reasoner_id;
        state_msg.deliberative_state = aerials::msg::DeliberativeState::CREATED;
        state_publisher->publish(state_msg);
    }

    void deliberative_manager::start_execution([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::Executor::Request> req, std::shared_ptr<aerials::srv::Executor::Response> res)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Staarting execution for reasoner %lu..", req->reasoner_id);
        if (const auto exec = executors.find(req->reasoner_id); exec != executors.end())
        {
            exec->second->start_execution(req->notify_start, req->notify_end);
            res->new_state = exec->second->state;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Reasoner %lu does not exists..", req->reasoner_id);
            res->new_state = aerials::msg::DeliberativeState::DESTROYED;
        }
    }

    void deliberative_manager::destroy_reasoner([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<aerials::srv::ReasonerDestroyer::Request> req, std::shared_ptr<aerials::srv::ReasonerDestroyer::Response> res)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Destroying reasoner %lu..", req->reasoner_id);
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
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Reasoner %lu does not exists..", req->reasoner_id);
            res->destroyed = false;
        }
    }
} // namespace ratio::ros
