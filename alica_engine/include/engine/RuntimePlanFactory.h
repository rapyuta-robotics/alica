#pragma once

#include <iostream>
#include <memory>

#include <condition_variable>
#include <mutex>

namespace alica
{

class Plan;
class BasicPlan;
class IPlanCreator;
class Blackboard;
class IAlicaTraceFactory;
class TeamManager;
class IAlicaTimerFactory;

class RuntimePlanFactory
{
public:
    RuntimePlanFactory(std::shared_ptr<Blackboard> globalBlackboard, const IAlicaTraceFactory* traceFactory, const TeamManager& teamManager,
            const IAlicaTimerFactory& timerFactory);
    ~RuntimePlanFactory() = default;
    void init(std::unique_ptr<IPlanCreator>&& pc);

    std::unique_ptr<BasicPlan> create(int64_t id, const Plan* planModel) const;

private:
    std::unique_ptr<IPlanCreator> _creator;
    std::shared_ptr<Blackboard> _globalBlackboard;
    const IAlicaTraceFactory* _traceFactory;
    const TeamManager& _teamManager;
    const IAlicaTimerFactory& _timerFactory;

    mutable std::mutex _m;
    mutable std::condition_variable _cv;
    bool _initialized{false};
};

} /* namespace alica */
