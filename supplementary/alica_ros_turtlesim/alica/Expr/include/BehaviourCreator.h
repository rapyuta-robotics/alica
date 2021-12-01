#pragma once
#include <engine/IAlicaWorldModel.h>
#include <engine/IBehaviourCreator.h>

#include <iostream>
#include <memory>

namespace alica
{

class BasicBehaviour;

class BehaviourCreator : public IBehaviourCreator
{
public:
    BehaviourCreator();
    virtual ~BehaviourCreator();
    virtual std::shared_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, IAlicaWorldModel* wm);
};

} /* namespace alica */
