#pragma once

#include <iostream>
#include <memory>

namespace alica
{

class AlicaEngine;
class Behaviour;
class BasicBehaviour;
class IBehaviourCreator;
class IAlicaWorldModel;

/**
 * Construct a runtime BasicBehaviour instance based on the Behaviour model
 */
class RuntimeBehaviourFactory
{
public:
    // TODO: remove engine reference later
    RuntimeBehaviourFactory(IBehaviourCreator& bc, IAlicaWorldModel* wm, AlicaEngine* engine);
    ~RuntimeBehaviourFactory() = default;

    std::unique_ptr<BasicBehaviour> create(int64_t id, const Behaviour* behaviourModel) const;

private:
    IBehaviourCreator& _creator;
    IAlicaWorldModel* _wm;
    AlicaEngine* _engine;
};

} /* namespace alica */
