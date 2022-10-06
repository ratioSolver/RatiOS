#include "deliberative_executor.h"
#include "deliberative_manager.h"
#include "predicate.h"
#include "item.h"

namespace ratio::ros
{
    deliberative_executor::deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id, const std::vector<std::string> &domain_files, const std::vector<std::string> &requirements) : d_mngr(d_mngr), reasoner_id(id), slv(), exec(slv), dcl(*this), dsl(*this), del(*this)
    { // a new reasoner has just been created..
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

        set_state(aerials::msg::DeliberativeState::EXECUTING);
    }
    void deliberative_executor::pause_execution()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Pausing execution..", reasoner_id);
        set_state(aerials::msg::DeliberativeState::PAUSED);
    }
    void deliberative_executor::stop_execution()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Stopping execution..", reasoner_id);
        set_state(aerials::msg::DeliberativeState::STOPPED);
    }
    void deliberative_executor::tick()
    {
        if (pending_requirements)
        { // we solve the problem again..
            slv.solve();
            pending_requirements = false;
        }
        if (current_state == aerials::msg::DeliberativeState::EXECUTING)
            exec.tick();
    }
    void deliberative_executor::append_requirements(const std::vector<std::string> &requirements)
    {
        if (current_state == aerials::msg::DeliberativeState::INCONSISTENT)
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
    void deliberative_executor::lengthen_task(const uintptr_t &id, const semitone::rational &delay)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Lengthning task %s..", reasoner_id, current_tasks.at(id)->get_type().get_name().c_str());
        std::unordered_map<const ratio::core::atom *, semitone::rational> dey;
        dey[current_tasks.at(id)] = delay;
        if (!dey.empty())
            exec.dont_end_yet(dey);
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
        current_state = st;
        auto state_msg = aerials::msg::DeliberativeState();
        state_msg.reasoner_id = reasoner_id;
        state_msg.deliberative_state = current_state;
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

    aerials::msg::Task deliberative_executor::to_task(const ratio::core::atom &atm) const noexcept
    {
        aerials::msg::Task task;
        task.reasoner_id = reasoner_id;
        task.task_id = get_id(atm);
        task.task_name = atm.get_type().get_name();

        for (const auto &[name, xpr] : atm.get_vars())
            if (name != RATIO_START && name != RATIO_END && name != RATIO_AT && name != TAU_KW)
            {
                task.par_names.push_back(name);
                if (auto bi = dynamic_cast<ratio::core::bool_item *>(&*xpr))
                    switch (atm.get_type().get_core().bool_value(*bi))
                    {
                    case semitone::True:
                        task.par_values.push_back("true");
                        break;
                    case semitone::False:
                        task.par_values.push_back("false");
                        break;
                    default:
                        break;
                    }
                else if (auto ai = dynamic_cast<ratio::core::arith_item *>(&*xpr))
                    task.par_values.push_back(to_string(atm.get_type().get_core().arith_value(*ai).get_rational()));
                else if (auto si = dynamic_cast<ratio::core::string_item *>(&*xpr))
                    task.par_values.push_back(si->get_value());
            }

        return task;
    }

    deliberative_executor::deliberative_core_listener::deliberative_core_listener(deliberative_executor &de) : core_listener(de.get_solver()), exec(de) {}
    void deliberative_executor::deliberative_core_listener::started_solving()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Started reasoning..", exec.reasoner_id);
        if (exec.previous_state == -1u)
            exec.set_state(aerials::msg::DeliberativeState::REASONING);
        else
        { // we remember the current state for restoring it after adaptation..
            assert(exec.current_state == aerials::msg::DeliberativeState::EXECUTING || exec.current_state == aerials::msg::DeliberativeState::PAUSED || exec.current_state == aerials::msg::DeliberativeState::STOPPED);
            exec.previous_state = exec.current_state;
            exec.set_state(aerials::msg::DeliberativeState::ADAPTING);
        }
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

        for (const auto &[id, atm] : exec.current_tasks)
            timelines_msg.executing.push_back(id);

        exec.d_mngr.timelines_publisher->publish(timelines_msg);

        switch (exec.previous_state)
        {
        case aerials::msg::DeliberativeState::EXECUTING:
            exec.set_state(aerials::msg::DeliberativeState::EXECUTING);
            break;
        case aerials::msg::DeliberativeState::PAUSED:
            exec.set_state(aerials::msg::DeliberativeState::PAUSED);
            break;
        case aerials::msg::DeliberativeState::STOPPED:
            exec.set_state(aerials::msg::DeliberativeState::STOPPED);
            break;
        default:
            break;
        }
    }
    void deliberative_executor::deliberative_core_listener::inconsistent_problem()
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Inconsistent problem..", exec.reasoner_id);

        exec.current_flaw = nullptr;
        exec.current_resolver = nullptr;

        exec.set_state(aerials::msg::DeliberativeState::INCONSISTENT);
    }

    deliberative_executor::deliberative_executor_listener::deliberative_executor_listener(deliberative_executor &de) : executor_listener(de.get_executor()), exec(de) {}
    void deliberative_executor::deliberative_executor_listener::tick(const semitone::rational &time)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Current time: %s", to_string(time).c_str());
        exec.current_time = time;

        auto timelines_msg = deliberative_tier::msg::Timelines();
        timelines_msg.reasoner_id = exec.reasoner_id;
        timelines_msg.update = deliberative_tier::msg::Timelines::TIME_CHANGED;
        timelines_msg.time.num = exec.current_time.numerator();
        timelines_msg.time.den = exec.current_time.denominator();
        exec.d_mngr.timelines_publisher->publish(timelines_msg);

        auto horizon = exec.slv.ratio::core::env::get("horizon");
        if (exec.slv.ratio::core::core::arith_value(horizon) <= exec.exec.get_current_time() && exec.current_tasks.empty())
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[%lu] Exhausted plan..", exec.reasoner_id);
            exec.set_state(aerials::msg::DeliberativeState::FINISHED);
        }
    }
    void deliberative_executor::deliberative_executor_listener::starting(const std::unordered_set<ratio::core::atom *> &atms)
    {
        auto request = std::make_shared<aerials::srv::TaskExecutor::Request>();
        std::vector<std::pair<rclcpp::Client<aerials::srv::TaskExecutor>::FutureAndRequestId, ratio::core::atom *>> futures;
        futures.reserve(atms.size());
        for (const auto &atm : atms)
            if (exec.notify_start.count(static_cast<ratio::core::predicate *>(&atm->get_type())))
            {
                request->task = exec.to_task(*atm);
                futures.push_back({std::move(exec.d_mngr.can_start->async_send_request(request)), atm});
            }
        // we tell the executor the atoms which are not yet ready to start..
        std::unordered_map<const ratio::core::atom *, semitone::rational> dsy;
        for (auto &[ftr, atm] : futures)
            if (rclcpp::spin_until_future_complete(exec.d_mngr.node, ftr) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result = ftr.get();
                if (!result->success)
                    dsy[atm] = result->delay.den == 0 ? semitone::rational(1) : semitone::rational(result->delay.num, result->delay.den);
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service `%s`", exec.d_mngr.can_start->get_service_name());
            }

        if (!dsy.empty())
            exec.exec.dont_start_yet(dsy);
    }
    void deliberative_executor::deliberative_executor_listener::start(const std::unordered_set<ratio::core::atom *> &atms)
    {
        auto request = std::make_shared<aerials::srv::TaskExecutor::Request>();
        std::vector<std::pair<rclcpp::Client<aerials::srv::TaskExecutor>::FutureAndRequestId, ratio::core::atom *>> futures;
        futures.reserve(atms.size());
        for (const auto &atm : atms)
            if (exec.notify_start.count(static_cast<ratio::core::predicate *>(&atm->get_type())))
            {
                request->task = exec.to_task(*atm);
                futures.push_back({std::move(exec.d_mngr.start_task->async_send_request(request)), atm});
            }
            else if (exec.notify_end.count(static_cast<ratio::core::predicate *>(&atm->get_type())))
                exec.current_tasks.emplace(get_id(*atm), atm);
        for (auto &[ftr, atm] : futures)
            if (rclcpp::spin_until_future_complete(exec.d_mngr.node, ftr) == rclcpp::FutureReturnCode::SUCCESS)
            {
                if (ftr.get()->success)
                    exec.current_tasks.emplace(get_id(*atm), atm);
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to start task `%s`", atm->get_type().get_name().c_str());
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service `%s`", exec.d_mngr.end_task->get_service_name());
            }

        auto timelines_msg = deliberative_tier::msg::Timelines();
        timelines_msg.reasoner_id = exec.reasoner_id;
        timelines_msg.update = deliberative_tier::msg::Timelines::EXECUTING_CHANGED;
        for (const auto &[id, atm] : exec.current_tasks)
            timelines_msg.executing.push_back(id);
        exec.d_mngr.timelines_publisher->publish(timelines_msg);
    }

    void deliberative_executor::deliberative_executor_listener::ending(const std::unordered_set<ratio::core::atom *> &atms)
    {
        auto request = std::make_shared<aerials::srv::TaskExecutor::Request>();
        std::vector<std::pair<rclcpp::Client<aerials::srv::TaskExecutor>::FutureAndRequestId, ratio::core::atom *>> futures;
        futures.reserve(atms.size());
        for (const auto &atm : atms)
            if (exec.notify_end.count(static_cast<ratio::core::predicate *>(&atm->get_type())))
            {
                request->task = exec.to_task(*atm);
                futures.push_back({std::move(exec.d_mngr.can_end->async_send_request(request)), atm});
            }
        // we tell the executor the atoms which are not yet ready to end..
        std::unordered_map<const ratio::core::atom *, semitone::rational> dey;
        for (auto &[ftr, atm] : futures)
            if (rclcpp::spin_until_future_complete(exec.d_mngr.node, ftr) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result = ftr.get();
                if (!result->success)
                    dey[atm] = result->delay.den == 0 ? semitone::rational(1) : semitone::rational(result->delay.num, result->delay.den);
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service `%s`", exec.d_mngr.can_end->get_service_name());
            }

        if (!dey.empty())
            exec.exec.dont_end_yet(dey);
    }
    void deliberative_executor::deliberative_executor_listener::end(const std::unordered_set<ratio::core::atom *> &atms)
    {
        auto request = std::make_shared<aerials::srv::TaskExecutor::Request>();
        std::vector<std::pair<rclcpp::Client<aerials::srv::TaskExecutor>::FutureAndRequestId, ratio::core::atom *>> futures;
        futures.reserve(atms.size());
        for (const auto &atm : atms)
            if (exec.notify_end.count(static_cast<ratio::core::predicate *>(&atm->get_type())))
            {
                request->task = exec.to_task(*atm);
                futures.push_back({std::move(exec.d_mngr.end_task->async_send_request(request)), atm});
            }
        for (auto &[ftr, atm] : futures)
            if (rclcpp::spin_until_future_complete(exec.d_mngr.node, ftr) == rclcpp::FutureReturnCode::SUCCESS)
            {
                if (!ftr.get()->success)
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to end task `%s`", atm->get_type().get_name().c_str());
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service `%s`", exec.d_mngr.end_task->get_service_name());
            }

        // these atoms are now ended..
        for (const auto &atm : atms)
            exec.current_tasks.erase(get_id(*atm));

        auto timelines_msg = deliberative_tier::msg::Timelines();
        timelines_msg.reasoner_id = exec.reasoner_id;
        timelines_msg.update = deliberative_tier::msg::Timelines::EXECUTING_CHANGED;
        for (const auto &[id, atm] : exec.current_tasks)
            timelines_msg.executing.push_back(id);
        exec.d_mngr.timelines_publisher->publish(timelines_msg);
    }

    deliberative_executor::deliberative_solver_listener::deliberative_solver_listener(deliberative_executor &de) : solver_listener(de.get_solver()), exec(de) {}
    void deliberative_executor::deliberative_solver_listener::flaw_created(const ratio::solver::flaw &f)
    {
        exec.flaws.insert(&f);

        auto fc_msg = deliberative_tier::msg::Flaw();
        fc_msg.id = get_id(f);
        for (const auto &r : f.get_causes())
            fc_msg.causes.push_back(get_id(*r));
        fc_msg.data = f.get_data().dump();
        fc_msg.state = slv.get_sat_core()->value(f.get_phi());
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        fc_msg.pos.lb = lb, fc_msg.pos.ub = ub;

        auto g_msg = deliberative_tier::msg::Graph();
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaws.push_back(fc_msg);
        g_msg.update = deliberative_tier::msg::Graph::FLAW_CREATED;
        exec.d_mngr.graph_publisher->publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::flaw_state_changed(const ratio::solver::flaw &f)
    {
        auto fsc_msg = deliberative_tier::msg::Flaw();
        fsc_msg.id = get_id(f);
        fsc_msg.state = slv.get_sat_core()->value(f.get_phi());

        auto g_msg = deliberative_tier::msg::Graph();
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaws.push_back(fsc_msg);
        g_msg.update = deliberative_tier::msg::Graph::FLAW_STATE_CHANGED;
        exec.d_mngr.graph_publisher->publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::flaw_cost_changed(const ratio::solver::flaw &f)
    {
        auto fcc_msg = deliberative_tier::msg::Flaw();
        fcc_msg.id = get_id(f);
        const auto est_cost = f.get_estimated_cost();
        fcc_msg.cost.num = est_cost.numerator(), fcc_msg.cost.den = est_cost.denominator();

        auto g_msg = deliberative_tier::msg::Graph();
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaws.push_back(fcc_msg);
        g_msg.update = deliberative_tier::msg::Graph::FLAW_COST_CHANGED;
        exec.d_mngr.graph_publisher->publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::flaw_position_changed(const ratio::solver::flaw &f)
    {
        auto fpc_msg = deliberative_tier::msg::Flaw();
        fpc_msg.id = get_id(f);
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        fpc_msg.pos.lb = lb, fpc_msg.pos.ub = ub;

        auto g_msg = deliberative_tier::msg::Graph();
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaws.push_back(fpc_msg);
        g_msg.update = deliberative_tier::msg::Graph::FLAW_POSITION_CHANGED;
        exec.d_mngr.graph_publisher->publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::current_flaw(const ratio::solver::flaw &f)
    {
        exec.current_flaw = &f;

        auto g_msg = deliberative_tier::msg::Graph();
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaw_id = get_id(f);
        g_msg.update = deliberative_tier::msg::Graph::CURRENT_FLAW;
        exec.d_mngr.graph_publisher->publish(g_msg);
    }

    void deliberative_executor::deliberative_solver_listener::resolver_created(const ratio::solver::resolver &r)
    {
        exec.resolvers.insert(&r);

        auto rc_msg = deliberative_tier::msg::Resolver();
        rc_msg.id = get_id(r);
        for (const auto &p : r.get_preconditions())
            rc_msg.preconditions.push_back(get_id(*p));
        rc_msg.effect = get_id(r.get_effect());
        rc_msg.data = r.get_data().dump();
        rc_msg.state = slv.get_sat_core()->value(r.get_rho());
        const auto est_cost = r.get_intrinsic_cost();
        rc_msg.intrinsic_cost.num = est_cost.numerator(), rc_msg.intrinsic_cost.den = est_cost.denominator();

        auto g_msg = deliberative_tier::msg::Graph();
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.resolvers.push_back(rc_msg);
        g_msg.update = deliberative_tier::msg::Graph::RESOLVER_CREATED;
        exec.d_mngr.graph_publisher->publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::resolver_state_changed(const ratio::solver::resolver &r)
    {
        auto rsc_msg = deliberative_tier::msg::Resolver();
        rsc_msg.id = get_id(r);
        rsc_msg.state = slv.get_sat_core()->value(r.get_rho());

        auto g_msg = deliberative_tier::msg::Graph();
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.resolvers.push_back(rsc_msg);
        g_msg.update = deliberative_tier::msg::Graph::RESOLVER_STATE_CHANGED;
        exec.d_mngr.graph_publisher->publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::current_resolver(const ratio::solver::resolver &r)
    {
        exec.current_resolver = &r;

        auto g_msg = deliberative_tier::msg::Graph();
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.resolver_id = get_id(r);
        g_msg.update = deliberative_tier::msg::Graph::CURRENT_RESOLVER;
        exec.d_mngr.graph_publisher->publish(g_msg);
    }

    void deliberative_executor::deliberative_solver_listener::causal_link_added(const ratio::solver::flaw &f, const ratio::solver::resolver &r)
    {
        auto g_msg = deliberative_tier::msg::Graph();
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaw_id = get_id(f);
        g_msg.resolver_id = get_id(r);
        g_msg.update = deliberative_tier::msg::Graph::CAUSAL_LINK_ADDED;
        exec.d_mngr.graph_publisher->publish(g_msg);
    }
} // namespace ratio::ros
