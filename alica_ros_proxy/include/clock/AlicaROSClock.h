#pragma

#include <engine/IAlicaClock.h>

namespace alicaRosProxy {

class AlicaROSClock : public virtual alica::IAlicaClock {
public:
    AlicaROSClock();
    virtual ~AlicaROSClock();
    virtual alica::AlicaTime now();
    virtual void sleep(long us);
};

} /* namespace alicaRosProxy */
