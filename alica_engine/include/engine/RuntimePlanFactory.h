#pragma once

#include <iostream>
#include <memory>

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
    RuntimePlanFactory(Blackboard& worldModels, const IAlicaTraceFactory* traceFactory, const TeamManager& teamManager, const IAlicaTimerFactory& timerFactory);
    ~RuntimePlanFactory() = default;
    void init(std::unique_ptr<IPlanCreator>&& pc);

    std::unique_ptr<BasicPlan> create(int64_t id, const Plan* planModel) const;

private:
    std::unique_ptr<IPlanCreator> _creator;
    Blackboard& _worldModels;
    const IAlicaTraceFactory* _traceFactory;
    const TeamManager& _teamManager;
    const IAlicaTimerFactory& _timerFactory;
};

} /* namespace alica */
