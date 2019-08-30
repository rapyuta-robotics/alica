#include <clock/AlicaROSClock.h>

namespace alicaRosProxy
{

AlicaROSClock::AlicaROSClock(alica::AlicaEngine* ae)
: AlicaClock(ae) {}

alica::AlicaTime AlicaROSClock::now() const
{
    return alica::AlicaTime::nanoseconds(ros::Time::now().toNSec());
}

void AlicaROSClock::sleep(const alica::AlicaTime& time) const
{
    ros::Duration(time.inSeconds()).sleep();
}

} // namespace alicaRosProxy
