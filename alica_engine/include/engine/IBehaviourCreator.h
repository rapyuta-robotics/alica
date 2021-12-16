#pragma once

#include "engine/IAlicaWorldModel.h"
#include <memory>

namespace alica
{
class AlicaEngine;
class BasicBehaviour;
class Configuration;

class IBehaviourCreator
{
public:
    virtual ~IBehaviourCreator() {}
    virtual std::shared_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, IAlicaWorldModel* wm) = 0;
};

} /* namespace alica */
