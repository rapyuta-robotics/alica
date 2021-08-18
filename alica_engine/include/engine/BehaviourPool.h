#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <typeindex>

namespace alica
{
class AlicaEngine;
class Behaviour;
class Configuration;
class BasicBehaviour;
class ConfAbstractPlanWrapper;
class IBehaviourCreator;
class RunningPlan;

/**
 * Manages the connection between the domain specific implementation (BasicBehaviours) of Behaviours.
 * It creates used BasicBehaviours with its given BehaviourCreator and starts and stops the Behaviours.
 */
class BehaviourPool
{
public:
    BehaviourPool(AlicaEngine* ae);
    ~BehaviourPool();
    bool init(IBehaviourCreator& bc);
    void startBehaviour(RunningPlan& rp);
    void stopBehaviour(RunningPlan& rp);
    void stopAll();
    void terminateAll();
    bool isBehaviourRunningInContext(const RunningPlan& rp) const;
    const std::map<const ConfAbstractPlanWrapper*, std::shared_ptr<BasicBehaviour>>& getAvailableBehaviours() const { return _availableBehaviours; }

private:
    const std::shared_ptr<BasicBehaviour> getBasicBehaviour(const Behaviour* behaviour, const Configuration* configuration) const;
    /**
     * Manages behaviours used by the running ALICA program.
     * The key of the map is the ConfAbstractPlanWrapper, which is implicitly created through the PlanDesigner.
     * The value is the basic behaviour, which is the implementation of that behaviour.
     */
    std::map<const ConfAbstractPlanWrapper*, std::shared_ptr<BasicBehaviour>> _availableBehaviours;
    AlicaEngine* _ae;
};

} /* namespace alica */
