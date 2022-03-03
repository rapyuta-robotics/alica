#pragma once

#include "engine/IAlicaWorldModel.h"
#include <memory>

namespace alica
{
class AlicaEngine;
class BasicBehaviour;
class Behaviour;

class IBehaviourCreator
{
public:
    virtual ~IBehaviourCreator() {}
    virtual std::unique_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, const Behaviour* behaviourModel, IAlicaWorldModel* wm) = 0;
};

} /* namespace alica */
