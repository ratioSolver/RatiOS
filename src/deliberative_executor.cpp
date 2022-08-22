#include "deliberative_executor.h"
#include "deliberative_manager.h"
#include "predicate.h"
#include "item.h"

namespace ratio::ros
{
    deliberative_executor::deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id, const std::vector<std::string> &domain_files, const std::vector<std::string> &requirements) : d_mngr(d_mngr), reasoner_id(id), slv(), exec(slv), dcl(*this), dsl(*this), del(*this) {}

    void deliberative_executor::start_execution(const std::vector<std::string> &notify_start_ids, const std::vector<std::string> &notify_end_ids)
    {
    }
    void deliberative_executor::tick()
    {
        if (pending_requirements)
        { // we solve the problem again..
            slv.solve();
            pending_requirements = false;
        }
        if (state == aerials::msg::DeliberativeState::EXECUTING)
            exec.tick();
    }
    void deliberative_executor::append_requirements(const std::vector<std::string> &requirements)
    {
        try
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Going at root level..", reasoner_id);
            while (!slv.root_level()) // we go at root level..
                slv.get_sat_core()->pop();
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Reading requirements..", reasoner_id);
            for (const auto &requirement : requirements)
            {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] %s", reasoner_id, requirement.c_str());
                slv.read(requirement);
            }

            pending_requirements = true;
        }
        catch (const ratio::core::inconsistency_exception &e)
        { // the problem is inconsistent..
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] The problem is inconsistent..", reasoner_id);
            set_state(aerials::msg::DeliberativeState::INCONSISTENT);
        }
        catch (const ratio::core::unsolvable_exception &e)
        { // the problem is unsolvable..
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] The problem is unsolvable..", reasoner_id);
            set_state(aerials::msg::DeliberativeState::INCONSISTENT);
        }
    }
    void deliberative_executor::close_task(const uintptr_t &id, const bool &success)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Closing task %s..", reasoner_id, current_tasks.at(id)->get_type().get_name().c_str());
        if (!success) // the task failed..
            exec.failure({current_tasks.at(id)});
        current_tasks.erase(id);
    }

    void deliberative_executor::set_state(const unsigned int &st)
    {
        state = st;
        auto state_msg = aerials::msg::DeliberativeState();
        state_msg.reasoner_id = reasoner_id;
        state_msg.deliberative_state = state;
        d_mngr.state_publisher->publish(state_msg);
    }

    deliberative_executor::deliberative_core_listener::deliberative_core_listener(deliberative_executor &de) : core_listener(de.get_solver()), exec(de) {}
    void deliberative_executor::deliberative_core_listener::started_solving()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Started reasoning..", exec.reasoner_id);
    }
    void deliberative_executor::deliberative_core_listener::solution_found()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Solution found..", exec.reasoner_id);
    }
    void deliberative_executor::deliberative_core_listener::inconsistent_problem()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Inconsistent problem..", exec.reasoner_id);
    }

    deliberative_executor::deliberative_executor_listener::deliberative_executor_listener(deliberative_executor &de) : executor_listener(de.get_executor()), exec(de) {}
    void deliberative_executor::deliberative_executor_listener::tick(const semitone::rational &time)
    {
    }
    void deliberative_executor::deliberative_executor_listener::starting(const std::unordered_set<ratio::core::atom *> &atms)
    {
    }
    void deliberative_executor::deliberative_executor_listener::start(const std::unordered_set<ratio::core::atom *> &atms)
    {
    }

    void deliberative_executor::deliberative_executor_listener::ending(const std::unordered_set<ratio::core::atom *> &atms)
    {
    }
    void deliberative_executor::deliberative_executor_listener::end(const std::unordered_set<ratio::core::atom *> &atms)
    {
    }

    deliberative_executor::deliberative_solver_listener::deliberative_solver_listener(deliberative_executor &de) : solver_listener(de.get_solver()), exec(de) {}
    void deliberative_executor::deliberative_solver_listener::flaw_created(const ratio::solver::flaw &f)
    {
    }
    void deliberative_executor::deliberative_solver_listener::flaw_state_changed(const ratio::solver::flaw &f)
    {
    }
    void deliberative_executor::deliberative_solver_listener::flaw_cost_changed(const ratio::solver::flaw &f)
    {
    }
    void deliberative_executor::deliberative_solver_listener::flaw_position_changed(const ratio::solver::flaw &f)
    {
    }
    void deliberative_executor::deliberative_solver_listener::current_flaw(const ratio::solver::flaw &f)
    {
    }

    void deliberative_executor::deliberative_solver_listener::resolver_created(const ratio::solver::resolver &r)
    {
    }
    void deliberative_executor::deliberative_solver_listener::resolver_state_changed(const ratio::solver::resolver &r)
    {
    }
    void deliberative_executor::deliberative_solver_listener::current_resolver(const ratio::solver::resolver &r)
    {
    }

    void deliberative_executor::deliberative_solver_listener::causal_link_added(const ratio::solver::flaw &f, const ratio::solver::resolver &r)
    {
    }
} // namespace ratio::ros
