#pragma once
#include <engine/IBehaviourCreator.h>

#include <iostream>
#include <memory>

namespace alica
{

class BasicBehaviour;

class DynamicBehaviourCreator : public IBehaviourCreator
{
public:
    DynamicBehaviourCreator();
    virtual ~DynamicBehaviourCreator();
    virtual std::unique_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, BehaviourContext& context) override;
};

} /* namespace alica */
