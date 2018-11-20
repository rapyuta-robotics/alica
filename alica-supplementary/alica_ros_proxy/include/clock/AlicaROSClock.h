#pragma once

#include <engine/AlicaClock.h>

namespace alicaRosProxy
{

class AlicaROSClock : public virtual alica::AlicaClock
{
public:
    AlicaROSClock();
    virtual ~AlicaROSClock();
    virtual alica::AlicaTime now() const override;
};

} /* namespace alicaRosProxy */
