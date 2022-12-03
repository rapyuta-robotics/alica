#pragma once

#include <iostream>
#include <memory>

namespace alica
{

class Behaviour;
class BasicBehaviour;
class IBehaviourCreator;
class Blackboard;
class TeamManager;
class PlanBase;
class IAlicaCommunication;
class IAlicaTraceFactory;
class IAlicaTimerFactory;
/**
 * Construct a runtime BasicBehaviour instance based on the Behaviour model
 */
class RuntimeBehaviourFactory
{
public:
    RuntimeBehaviourFactory(Blackboard& worldModels, TeamManager& teamManager, PlanBase& planBase, const IAlicaCommunication& communication,
            const IAlicaTraceFactory* traceFactory, const IAlicaTimerFactory& timerFactory);
    ~RuntimeBehaviourFactory() = default;
    void init(std::unique_ptr<IBehaviourCreator>&& bc);

    std::unique_ptr<BasicBehaviour> create(int64_t id, const Behaviour* behaviourModel) const;

private:
    std::unique_ptr<IBehaviourCreator> _creator;
    Blackboard& _worldModels;
    TeamManager& _teamManager;
    PlanBase& _planBase;
    const IAlicaCommunication& _communication;
    const IAlicaTraceFactory* _traceFactory;
    const IAlicaTimerFactory& _timerFactory;
};

} /* namespace alica */
