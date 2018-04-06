#include "engine/AlicaClock.h"

namespace alica {

AlicaClock::AlicaClock() {}
virtual AlicaClock::~AlicaClock() {}

virtual AlicaTime AlicaClock::AlicaClock::now() {
    return AlicaTime(std::chrono::steady_clock()::now());
}

void AlicaClock::AlicaClock::sleep(long us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

/* BEFORE
alica::AlicaTime AlicaSystemClock::now() {
    time_t t;
    t = time(NULL);
    alica::AlicaTime ret = (alica::AlicaTime)(t * 1000000000UL + t * 1000 * 1000);
    return ret;
}

void AlicaSystemClock::sleep(long us) {
    int sec = us / 1000000;
    int nsec = (us % 1000000) * 1000;
    ::sleep(sec * 1000);
}
*/
}