#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <typeindex>

namespace alica
{

class BehaviourConfiguration;
class BasicBehaviour;
class AlicaEngine;
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
    bool init(IBehaviourCreator* bc);
    void startBehaviour(RunningPlan& rp);
    void stopBehaviour(RunningPlan& rp);
    void stopAll();
    bool isBehaviourRunningInContext(const RunningPlan& rp) const;
    const std::map<const BehaviourConfiguration*, std::shared_ptr<BasicBehaviour>>& getAvailableBehaviours() const { return _availableBehaviours; }

private:
    /**
     * Manages behaviours used by the running ALICA program.
     * The key of the map is the behaviour configuration, which is created through the plan designer.
     * The value is the basic behaviour, which is the implementation of that behaviour.
     */
    // TODO: switch to unique ptr
    std::map<const BehaviourConfiguration*, std::shared_ptr<BasicBehaviour>> _availableBehaviours;
    AlicaEngine* _ae;
    IBehaviourCreator* _behaviourCreator;
};

} /* namespace alica */
