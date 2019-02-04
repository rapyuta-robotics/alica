#include "engine/AlicaClock.h"

#include <chrono>
#include <thread>

namespace alica
{

AlicaTime AlicaClock::now() const
{
    return AlicaTime::nanoseconds(std::chrono::system_clock::now().time_since_epoch().count());
}

void AlicaClock::sleep(const AlicaTime& time) const
{
    std::this_thread::sleep_for(std::chrono::nanoseconds(time.inNanoseconds()));
}

std::ostream& operator<<(std::ostream& os, const AlicaTime& time)
{
    return os << time.inNanoseconds();
}
}
