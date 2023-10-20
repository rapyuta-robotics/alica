#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <atomic>
#include <memory>

namespace ros_utils
{

class WaitForMsg : public alica::BasicBehaviour
{
public:
    WaitForMsg(alica::BehaviourContext& context);
    void initialiseParameters() override;
    static std::unique_ptr<WaitForMsg> create(alica::BehaviourContext& context);

private:
    void onMsg(const std_msgs::String& triggerMsg);

    std::string _topic;
    ros::Subscriber _sub;
};
BOOST_DLL_ALIAS(ros_utils::WaitForMsg::create, WaitForMsg)

} /* namespace ros_utils */
