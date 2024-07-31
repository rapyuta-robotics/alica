#pragma once

#include <iostream>
#include <memory>

#include <condition_variable>
#include <mutex>

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
    RuntimeBehaviourFactory(std::shared_ptr<Blackboard> globalBlackboard, TeamManager& teamManager, PlanBase& planBase,
            const IAlicaCommunication& communication, const IAlicaTraceFactory* traceFactory, const IAlicaTimerFactory& timerFactory);
    ~RuntimeBehaviourFactory() = default;
    void init(std::unique_ptr<IBehaviourCreator>&& bc);

    std::unique_ptr<BasicBehaviour> create(int64_t id, const Behaviour* behaviourModel) const;

private:
    std::unique_ptr<IBehaviourCreator> _creator;
    std::shared_ptr<Blackboard> _globalBlackboard;
    TeamManager& _teamManager;
    PlanBase& _planBase;
    const IAlicaCommunication& _communication;
    const IAlicaTraceFactory* _traceFactory;
    const IAlicaTimerFactory& _timerFactory;

    mutable std::mutex _m;
    mutable std::condition_variable _cv;
    bool _initialized{false};
};

} /* namespace alica */
