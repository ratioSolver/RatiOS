#pragma once

#include "executor_listener.h"
#include "aerials/Task.h"
#include "aerials/DeliberativeState.h"

namespace ratio::ros1
{
  class deliberative_manager;

  class deliberative_executor
  {
    friend class deliberative_manager;

  public:
    deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id, const std::vector<std::string> &domain_files, const std::vector<std::string> &requirements);

    uint64_t get_reasoner_id() { return reasoner_id; }
    ratio::solver::solver &get_solver() { return slv; }
    ratio::executor::executor &get_executor() { return exec; }

    void start_execution(const std::vector<std::string> &notify_start_ids, const std::vector<std::string> &notify_end_ids);
    void pause_execution();
    void tick();
    void append_requirements(const std::vector<std::string> &requirements);
    void delay_task(const uintptr_t &id, const semitone::rational &delay = semitone::rational(1));
    void extend_task(const uintptr_t &id, const semitone::rational &delay = semitone::rational(1));
    void close_task(const uintptr_t &id, const bool &success = true);

  private:
    void set_state(const unsigned int &state);
    ratio::core::predicate &get_predicate(const std::string &pred) const;

    aerials::Task to_task(const ratio::core::atom &atm) const noexcept;

  private:
    class deliberative_core_listener : public ratio::core::core_listener
    {
    public:
      deliberative_core_listener(deliberative_executor &de);

    private:
      void started_solving() override;
      void solution_found() override;
      void inconsistent_problem() override;

    private:
      deliberative_executor &exec;
    };

    class deliberative_executor_listener : public ratio::executor::executor_listener
    {
    public:
      deliberative_executor_listener(deliberative_executor &de);

    private:
      void tick(const semitone::rational &time) override;

      void starting(const std::unordered_set<ratio::core::atom *> &atms) override;
      void start(const std::unordered_set<ratio::core::atom *> &atms) override;

      void ending(const std::unordered_set<ratio::core::atom *> &atms) override;
      void end(const std::unordered_set<ratio::core::atom *> &atms) override;

    private:
      deliberative_executor &exec;
    };

    class deliberative_solver_listener : public ratio::solver::solver_listener
    {
    public:
      deliberative_solver_listener(deliberative_executor &de);

    private:
      void flaw_created(const ratio::solver::flaw &f) override;
      void flaw_state_changed(const ratio::solver::flaw &f) override;
      void flaw_cost_changed(const ratio::solver::flaw &f) override;
      void flaw_position_changed(const ratio::solver::flaw &f) override;
      void current_flaw(const ratio::solver::flaw &f) override;

      void resolver_created(const ratio::solver::resolver &r) override;
      void resolver_state_changed(const ratio::solver::resolver &r) override;
      void current_resolver(const ratio::solver::resolver &r) override;

      void causal_link_added(const ratio::solver::flaw &f, const ratio::solver::resolver &r) override;

    private:
      deliberative_executor &exec;
    };

  private:
    deliberative_manager &d_mngr;
    uint64_t reasoner_id;
    ratio::solver::solver slv;
    ratio::executor::executor exec;
    deliberative_core_listener dcl;
    deliberative_solver_listener dsl;
    deliberative_executor_listener del;
    unsigned int current_state;
    std::unordered_set<const ratio::core::predicate *> notify_start, notify_end;
    std::unordered_map<uintptr_t, const ratio::core::atom *> current_tasks;
    std::unordered_set<const ratio::solver::flaw *> flaws;
    std::unordered_set<const ratio::solver::resolver *> resolvers;
    const ratio::solver::flaw *current_flaw = nullptr;
    const ratio::solver::resolver *current_resolver = nullptr;
    semitone::rational current_time;
  };
} // namespace ratio::ros1
