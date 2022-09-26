#pragma once

#include <iostream>
#include <memory>

namespace alica
{

class Plan;
class BasicPlan;
class IPlanCreator;
class IAlicaWorldModel;
class IAlicaTraceFactory;
class TeamManager;
class IAlicaTimerFactory;

class RuntimePlanFactory
{
public:
    RuntimePlanFactory(IAlicaWorldModel* wm, const IAlicaTraceFactory* traceFactory, const TeamManager& teamManager, const IAlicaTimerFactory& timerFactory);
    ~RuntimePlanFactory() = default;
    void init(std::unique_ptr<IPlanCreator>&& pc);

    std::unique_ptr<BasicPlan> create(int64_t id, const Plan* planModel) const;

private:
    std::unique_ptr<IPlanCreator> _creator;
    IAlicaWorldModel* _wm;
    const IAlicaTraceFactory* _traceFactory;
    const TeamManager& _teamManager;
    const IAlicaTimerFactory& _timerFactory;
};

} /* namespace alica */
