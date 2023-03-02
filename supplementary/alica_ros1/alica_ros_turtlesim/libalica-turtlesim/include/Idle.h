#pragma once

#include <boost/dll/alias.hpp>
#include <engine/AlicaClock.h>
#include <engine/BasicBehaviour.h>

namespace turtlesim
{

class Idle : public alica::BasicBehaviour
{
public:
    Idle(alica::BehaviourContext& context);
    virtual ~Idle();
    virtual void initialiseParameters();
    virtual void run();
    static std::unique_ptr<Idle> create(alica::BehaviourContext& context);

protected:
    alica::AlicaTime _endIdleTime;
};

BOOST_DLL_ALIAS(turtlesim::Idle::create, Idle)
} /* namespace turtlesim */
