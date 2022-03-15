#pragma once

#include <memory>

namespace alica
{
class AlicaEngine;
class BasicBehaviour;
class BehaviourContext;

class IBehaviourCreator
{
public:
    virtual ~IBehaviourCreator() {}
    virtual std::unique_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, BehaviourContext& wm) = 0;
};

} /* namespace alica */
