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
class IAlicaTraceFactory;

/**
 * Construct a runtime BasicBehaviour instance based on the Behaviour model
 */
class RuntimeBehaviourFactory
{
public:
    // TODO: remove engine reference later
    RuntimeBehaviourFactory(std::unique_ptr<IBehaviourCreator>&& bc, IAlicaWorldModel* wm, const IAlicaTraceFactory* tf, AlicaEngine* engine);
    ~RuntimeBehaviourFactory() = default;

    std::unique_ptr<BasicBehaviour> create(int64_t id, const Behaviour* behaviourModel) const;

private:
    std::unique_ptr<IBehaviourCreator> _creator;
    IAlicaWorldModel* _wm;
    AlicaEngine* _engine;
    const IAlicaTraceFactory* _tf;
};

} /* namespace alica */
