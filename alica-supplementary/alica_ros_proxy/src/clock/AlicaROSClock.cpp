#include <clock/AlicaROSClock.h>
#include "ros/time.h"

namespace alicaRosProxy
{

AlicaROSClock::AlicaROSClock()
{
    if(!ros::Time::isValid()) {
        ros::Time::init();
    }
}

alica::AlicaTime AlicaROSClock::now() const
{
    return alica::AlicaTime::nanoseconds(ros::Time::now().toNSec());
}

void AlicaROSClock::sleep(const alica::AlicaTime& time) const
{
    ros::Duration(time.inSeconds()).sleep();
}

} // namespace alicaRosProxy
