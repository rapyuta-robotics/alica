#pragma once

#include <memory>

namespace alica
{
class BasicBehaviour;
class BehaviourContext;

class IBehaviourCreator
{
public:
    virtual ~IBehaviourCreator() {}
    virtual std::unique_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, BehaviourContext& context) = 0;
};

} /* namespace alica */
