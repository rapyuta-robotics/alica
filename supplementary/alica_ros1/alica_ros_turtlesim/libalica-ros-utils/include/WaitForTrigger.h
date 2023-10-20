#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <atomic>
#include <memory>

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
    void onTrigger(const std_msgs::Empty& triggerMsg);

    std::string _topic;
    std::atomic<bool> _triggered;
    ros::Subscriber _triggerSub;
};
BOOST_DLL_ALIAS(ros_utils::WaitForTrigger::create, WaitForTrigger)

} /* namespace ros_utils */
