#include "clock/AlicaROSClock.h"
#include "ros/time.h"

namespace alicaRosProxy {

AlicaROSClock::AlicaROSClock() {
    ros::Time::init();
}

AlicaROSClock::~AlicaROSClock() {}

alica::AlicaTime AlicaROSClock::now() {
    ros::Time t = ros::Time::now();
    alica::AlicaTime ret = (alica::AlicaTime)(t.sec * 1000000000UL + t.nsec);
    return ret;
}
void AlicaROSClock::sleep(long us) {
    int sec = us / 1000000;
    int nsec = (us % 1000000) * 1000;
    ros::Duration(sec, nsec).sleep();
}

}  // namespace alicaRosProxy
