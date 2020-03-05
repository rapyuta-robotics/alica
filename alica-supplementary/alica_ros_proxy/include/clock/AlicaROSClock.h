#pragma once

#include <engine/AlicaClock.h>

namespace alicaRosProxy
{

class AlicaROSClock : public alica::AlicaClock
{
public:
    AlicaROSClock();
    virtual ~AlicaROSClock() {}
    alica::AlicaTime now() const override;
    void sleep(const alica::AlicaTime&) const override;
};

} /* namespace alicaRosProxy */
