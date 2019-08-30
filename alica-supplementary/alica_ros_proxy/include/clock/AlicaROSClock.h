#pragma once

#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include "ros/ros.h"


namespace alicaRosProxy
{

class AlicaROSClock : public virtual alica::AlicaClock
{
public:
    AlicaROSClock(alica::AlicaEngine* ae);
    virtual ~AlicaROSClock() {};
    virtual alica::AlicaTime now() const override;
    virtual void sleep(const alica::AlicaTime&) const override;
};

} /* namespace alicaRosProxy */
