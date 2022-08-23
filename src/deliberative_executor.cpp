#include "deliberative_executor.h"
#include "deliberative_manager.h"
#include "predicate.h"
#include "item.h"

namespace ratio::ros
{
    deliberative_executor::deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id, const std::vector<std::string> &domain_files, const std::vector<std::string> &requirements) : d_mngr(d_mngr), reasoner_id(id), slv(), exec(slv), dcl(*this), dsl(*this), del(*this)
    {
        // a new reasoner has just been created..
        set_state(aerials::msg::DeliberativeState::CREATED);

        try
        {
            // we read the domain files..
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Reading domain..", reasoner_id);
            for (const auto &domain_file : domain_files)
            {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] %s", reasoner_id, domain_file.c_str());
            }
            slv.read(domain_files);

            // we read the requirements..
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

    void deliberative_executor::start_execution(const std::vector<std::string> &notify_start_ids, const std::vector<std::string> &notify_end_ids)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Starting execution..", reasoner_id);

        notify_start.clear();
        for (const auto &pred : notify_start_ids)
            notify_start.insert(&get_predicate(pred));
        notify_end.clear();
        for (const auto &pred : notify_end_ids)
            notify_end.insert(&get_predicate(pred));

        restart_execution = true;
        set_state(aerials::msg::DeliberativeState::EXECUTING);
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
        if (state == aerials::msg::DeliberativeState::INCONSISTENT)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Reasoner #%lu is inconsistent..", reasoner_id);
            return;
        }
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
    ratio::core::predicate &deliberative_executor::get_predicate(const std::string &pred) const
    {
        std::vector<std::string> ids;
        size_t start = 0, end = 0;
        do
        {
            end = pred.find('.', start);
            if (end == std::string::npos)
                end = pred.length();
            std::string token = pred.substr(start, end - start);
            if (!token.empty())
                ids.push_back(token);
            start = end + 1;
        } while (end < pred.length() && start < pred.length());

        if (ids.size() == 1)
            return slv.get_predicate(ids[0]);
        else
        {
            ratio::core::type *tp = &slv.get_type(ids[0]);
            for (size_t i = 1; i < ids.size(); ++i)
                if (i == ids.size() - 1)
                    return tp->get_predicate(ids[i]);
                else
                    tp = &tp->get_type(ids[i]);
        }
        // not found
        throw std::out_of_range(pred);
    }

    deliberative_executor::deliberative_core_listener::deliberative_core_listener(deliberative_executor &de) : core_listener(de.get_solver()), exec(de) {}
    void deliberative_executor::deliberative_core_listener::started_solving()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Started reasoning..", exec.reasoner_id);
        exec.set_state(aerials::msg::DeliberativeState::REASONING);
    }
    void deliberative_executor::deliberative_core_listener::solution_found()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Solution found..", exec.reasoner_id);
        exec.current_flaw = nullptr;
        exec.current_resolver = nullptr;

        auto timelines_msg = deliberative_tier::msg::Timelines();
        timelines_msg.reasoner_id = exec.reasoner_id;
        timelines_msg.update = deliberative_tier::msg::Timelines::STATE_CHANGED;

        std::stringstream sss;
        sss << ratio::solver::to_json(exec.slv).dump();
        timelines_msg.state = sss.str();

        const auto j_tls = ratio::solver::to_timelines(exec.slv);
        for (size_t i = 0; i < static_cast<json::array &>(j_tls).size(); ++i)
            timelines_msg.timelines.push_back(static_cast<json::array &>(j_tls)[i].dump());

        timelines_msg.time.num = exec.current_time.numerator();
        timelines_msg.time.den = exec.current_time.denominator();

        for (const auto &atm : exec.executing)
            timelines_msg.executing.push_back(get_id(*atm));

        exec.d_mngr.timelines_publisher->publish(timelines_msg);

        exec.set_state(exec.restart_execution ? aerials::msg::DeliberativeState::EXECUTING : aerials::msg::DeliberativeState::IDLE);
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
