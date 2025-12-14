#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <atomic>

namespace ros_utils
{

class WaitForTrigger : public alica::BasicBehaviour
{
public:
    WaitForTrigger(alica::BehaviourContext& context);
    void initialiseParameters() override;
    void run() override;
    static std::unique_ptr<WaitForTrigger> create(alica::BehaviourContext& context);

private:
    void onTrigger(const std_msgs::msg::Empty& triggerMsg);

    std::string _topic;
    std::atomic<bool> _triggered;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _triggerSub;
};
BOOST_DLL_ALIAS(ros_utils::WaitForTrigger::create, WaitForTrigger)

} /* namespace ros_utils */
