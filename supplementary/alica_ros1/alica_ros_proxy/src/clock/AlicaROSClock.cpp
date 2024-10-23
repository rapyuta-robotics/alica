#include "engine/AlicaClock.h"
#include "ros/time.h"
#include <clock/AlicaROSClock.h>

namespace alicaRosProxy
{

AlicaROSClock::AlicaROSClock()
{
    if (!ros::Time::isValid()) {
        ros::Time::init();
    }
}

alica::AlicaTime AlicaROSClock::now() const
{
    return alica::AlicaTime::nanoseconds(ros::Time::now().toNSec());
}

void AlicaROSClock::sleep(const alica::AlicaTime& time) const
{
    ros::Duration((double) time.inNanoseconds() / alica::AlicaTime::seconds(1).inNanoseconds()).sleep();
}

} // namespace alicaRosProxy
