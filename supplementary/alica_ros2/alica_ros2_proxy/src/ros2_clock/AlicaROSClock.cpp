#include <ros2_clock/AlicaROSClock.h>

#include <rclcpp/logging.hpp>

namespace alicaRosProxy
{

AlicaROSClock::AlicaROSClock()
{
    _rosTime = std::make_shared<rclcpp::Node>("rosTime");
    rclcpp::Parameter simTime("use_sim_time", rclcpp::ParameterValue(true));
    _rosTime->set_parameter(simTime);
}

alica::AlicaTime AlicaROSClock::now() const
{
    return alica::AlicaTime::nanoseconds((_rosTime->now()).nanoseconds());
}

void AlicaROSClock::sleep(const alica::AlicaTime& time) const
{
    // This will only work in Humble and beyond
    //_rosTime->get_clock()->sleep_for(rclcpp::Duration(time.inNanoseconds()));
}

} // namespace alicaRosProxy
