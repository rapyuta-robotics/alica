#pragma once

#include <iostream>
#include <memory>
#include <engine/PlanRepository.h>

namespace alica
{

class AlicaEngine;
class Behaviour;
class BasicBehaviour;
class IBehaviourCreator;
class IAlicaWorldModel;

/**
 * Construct a runtime BasicBehaviour instance based
 */
class RuntimeBehaviourFactory
{
public:
    // TODO: remove engine reference later
    RuntimeBehaviourFactory(IBehaviourCreator& bc, const PlanRepository::MapType<Behaviour>& behaviourRepo, IAlicaWorldModel *wm, AlicaEngine *engine);
    ~RuntimeBehaviourFactory() = default;

    std::unique_ptr<BasicBehaviour> create(int64_t id) const;
private:

    IBehaviourCreator& _creator;
    const PlanRepository::MapType<Behaviour>& _behaviourRepo;
    IAlicaWorldModel *_wm;
    AlicaEngine *_engine;
};

} /* namespace alica */
