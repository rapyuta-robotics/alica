#include <clock/AlicaROSClock.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <thread>

namespace alicaRosProxy
{

AlicaROSClock::AlicaROSClock(rclcpp::Node::SharedPtr node)
        : _clock(node ? node->get_clock() : std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
{
}

alica::AlicaTime AlicaROSClock::now() const
{
    return alica::AlicaTime::nanoseconds(_clock->now().nanoseconds());
}

void AlicaROSClock::sleep(const alica::AlicaTime& time) const
{
    try {
        _clock->sleep_for(rclcpp::Duration::from_nanoseconds(time.inNanoseconds()));
    } catch (const std::runtime_error&) {
        // The rclcpp context is not valid, for example after SIGINT. A short sleep, so that a caller that polls with
        // sleep() does not use a full core, and shutdown is not delayed.
        std::this_thread::sleep_for(std::chrono::nanoseconds(std::min<int64_t>(time.inNanoseconds(), 100000000)));
    }
}

} // namespace alicaRosProxy
