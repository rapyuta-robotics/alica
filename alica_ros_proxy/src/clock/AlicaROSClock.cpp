#include <clock/AlicaROSClock.h>

#include "ros/time.h"

namespace alicaRosProxy {

AlicaROSClock::AlicaROSClock() {
    ros::Time::init();
}

AlicaROSClock::~AlicaROSClock() {}

alica::AlicaTime AlicaROSClock::now() const {
    return alica::AlicaTime::nanoseconds(ros::Time::now().toNSec());
}

}  // namespace alicaRosProxy
