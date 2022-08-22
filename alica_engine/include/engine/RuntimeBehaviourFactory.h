#pragma once

#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace alica
{

class Behaviour;
class BasicBehaviour;
class IBehaviourCreator;
class IAlicaWorldModel;
class TeamManager;
class PlanBase;
class IAlicaCommunication;
class IAlicaTraceFactory;
class IAlicaTimerFactory;
class ConfigChangeListener;
/**
 * Construct a runtime BasicBehaviour instance based on the Behaviour model
 */
class RuntimeBehaviourFactory
{
public:
    RuntimeBehaviourFactory(ConfigChangeListener& configChangeListener, IAlicaWorldModel* wm, TeamManager& teamManager, PlanBase& planBase,
            const IAlicaCommunication& communication, const IAlicaTraceFactory* traceFactory, const IAlicaTimerFactory& timerFactory);
    ~RuntimeBehaviourFactory() = default;
    void init(std::unique_ptr<IBehaviourCreator>&& bc);

    std::unique_ptr<BasicBehaviour> create(int64_t id, const Behaviour* behaviourModel) const;

    void reload(const YAML::Node& config);

private:
    std::unique_ptr<IBehaviourCreator> _creator;
    IAlicaWorldModel* _wm;
    TeamManager& _teamManager;
    PlanBase& _planBase;
    const IAlicaCommunication& _communication;
    const IAlicaTraceFactory* _traceFactory;
    const IAlicaTimerFactory& _timerFactory;

    std::string _customerLibraryFolder;
};

} /* namespace alica */
