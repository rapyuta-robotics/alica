#pragma once

#include "alica_ros_turtlesim/turtle.hpp"
#include <boost/dll/alias.hpp>
#include <engine/AlicaContext.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace turtlesim
{
/*
        ALICATurtleWorldModel
        - WorldModel for ALICA ros turtlesim which interface between ALICA and ROS.
        - ROS:
                - Subscribe: t/init
*/

class ALICATurtleWorldModel
{
public:
    ALICATurtleWorldModel(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    ~ALICATurtleWorldModel();
    bool getInit() const { return _initTrigger; };
    void setInit(const bool input) { _initTrigger = input; };

    ALICATurtle turtle;

private:
    void initTriggerSubCallback(const std_msgs::EmptyConstPtr& msg); // callback of /init
    ros::Subscriber _initTriggerSub;                                 // user input for initialize,
    bool _initTrigger;                                               // become true when /init topic published
};

inline void setWorldModel(alica::AlicaContext* ac, ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
    alica::LockedBlackboardRW(ac->editGlobalBlackboard()).registerValue("worldmodel", std::make_shared<turtlesim::ALICATurtleWorldModel>(nh, priv_nh));
}

BOOST_DLL_ALIAS(turtlesim::setWorldModel, setWorldModel)

} // namespace turtlesim
