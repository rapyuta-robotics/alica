#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros_utils
{

class WaitForMsg : public alica::BasicBehaviour
{
public:
    WaitForMsg(alica::BehaviourContext& context);
    void initialiseParameters() override;
    static std::unique_ptr<WaitForMsg> create(alica::BehaviourContext& context);

private:
    void onMsg(const std_msgs::msg::String::SharedPtr msg);

    std::string _topic;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;
};
BOOST_DLL_ALIAS(ros_utils::WaitForMsg::create, WaitForMsg)

} /* namespace ros_utils */
