#pragma once

#include <engine/Types.h>

#include <iostream>
#include <memory>

namespace alica
{

class AlicaEngine;
class Behaviour;
class BasicBehaviour;
class IBehaviourCreator;
class IAlicaWorldModel;
class ConfigChangeListener;

/**
 * Construct a runtime BasicBehaviour instance based on the Behaviour model
 */
class RuntimeBehaviourFactory
{
public:
    // TODO: remove engine reference later
    RuntimeBehaviourFactory(ConfigChangeListener& configChangeListener, std::unique_ptr<IBehaviourCreator>&& bc, IAlicaWorldModel* wm, AlicaEngine* engine);
    ~RuntimeBehaviourFactory() = default;

    std::unique_ptr<BasicBehaviour> create(int64_t id, const Behaviour* behaviourModel) const;

    void reload(const YAML::Node& config);

private:
    std::unique_ptr<IBehaviourCreator> _creator;
    IAlicaWorldModel* _wm;
    AlicaEngine* _engine;
};

} /* namespace alica */
