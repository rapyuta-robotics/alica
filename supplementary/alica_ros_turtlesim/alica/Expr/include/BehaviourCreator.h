#pragma once
#include <engine/IBehaviourCreator.h>

#include <iostream>
#include <memory>

namespace alica
{

class BasicBehaviour;
class Behaviour;
class IAlicaWorldModel;

class BehaviourCreator : public IBehaviourCreator
{
public:
    BehaviourCreator();
    virtual ~BehaviourCreator();
    virtual std::unique_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, BehaviourContext& context) override;
};

} /* namespace alica */
