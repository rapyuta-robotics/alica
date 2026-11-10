#pragma once

#include <engine/AlicaClock.h>

#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>

namespace alicaRosProxy
{

/**
 * If a node is given, the clock of that node is used, so the time follows its use_sim_time parameter.
 * Else ROS time is used, which is the system time.
 */
class AlicaROSClock : public alica::AlicaClock
{
public:
    explicit AlicaROSClock(rclcpp::Node::SharedPtr node = nullptr);
    virtual ~AlicaROSClock() {}
    alica::AlicaTime now() const override;
    void sleep(const alica::AlicaTime&) const override;

private:
    rclcpp::Clock::SharedPtr _clock;
};

} // namespace alicaRosProxy
