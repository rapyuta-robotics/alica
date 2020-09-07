#pragma once

#include <memory>

namespace alica
{
class AlicaEngine;
class BasicBehaviour;

class IBehaviourCreator
{
public:
    virtual ~IBehaviourCreator() {}
    virtual std::shared_ptr<BasicBehaviour> createBehaviour(int64_t behaviourConfId) = 0;
};

} /* namespace alica */
