#include <clock2/AlicaROS2Clock.h>
#include <rclcpp/logging.hpp>

namespace alicaRos2Proxy
{

AlicaROS2Clock::AlicaROS2Clock()
{
    _rosTime = std::make_shared<rclcpp::Node>("rosTime");
    rclcpp::Parameter simTime("use_sim_time", rclcpp::ParameterValue(true));
    _rosTime->set_parameter(simTime);
}

alica::AlicaTime AlicaROS2Clock::now() const
{
    return alica::AlicaTime::nanoseconds((_rosTime->now()).nanoseconds());
}

void AlicaROS2Clock::sleep(const alica::AlicaTime& time) const
{
    // broken
    //_rosTime->get_clock()->sleep_for(rclcpp::Duration(time.inNanoseconds()));
}

} // namespace alicaRos2Proxy
