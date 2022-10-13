#pragma once

#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace alica
{

class Plan;
class BasicPlan;
class IPlanCreator;
class IAlicaWorldModel;
class ConfigChangeListener;
class IAlicaTraceFactory;
class TeamManager;
class IAlicaTimerFactory;

class RuntimePlanFactory
{
public:
    RuntimePlanFactory(ConfigChangeListener& configChangeListener, IAlicaWorldModel* wm, const IAlicaTraceFactory* traceFactory, const TeamManager& teamManager,
            const IAlicaTimerFactory& timerFactory);
    ~RuntimePlanFactory() = default;
    void init(std::unique_ptr<IPlanCreator>&& pc);

    std::unique_ptr<BasicPlan> create(int64_t id, const Plan* planModel) const;

    void reload(const YAML::Node& config);

private:
    std::unique_ptr<IPlanCreator> _creator;
    IAlicaWorldModel* _wm;
    const IAlicaTraceFactory* _traceFactory;
    const TeamManager& _teamManager;
    const IAlicaTimerFactory& _timerFactory;
};

} /* namespace alica */
