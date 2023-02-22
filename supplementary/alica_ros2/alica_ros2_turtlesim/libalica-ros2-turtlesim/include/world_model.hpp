#pragma once

#include "turtle.hpp"
#include <boost/dll/alias.hpp>
#include <engine/AlicaEngine.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

namespace turtlesim
{
/*
        ALICATurtleWorldModel
        - WorldModel for ALICA ros turtlesim which interface between ALICA and ROS.
        - A few class is static since one robot has one world model
        - ROS:
                - Subscribe: t/init
*/
class ALICATurtleWorldModel
{
public:
    static ALICATurtleWorldModel* get();                                           // return instance
    static void init(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr priv_nh); // create instance
    static void del();
    bool getInit() const { return _initTrigger; };
    void setInit(const bool input) { _initTrigger = input; };

    static ALICATurtleWorldModel* instance;
    ALICATurtle turtle;

private:
    ALICATurtleWorldModel(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr priv_nh);
    ~ALICATurtleWorldModel();
    void initTriggerSubCallback(const std_msgs::msg::Empty& msg);          // callback of /init
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _initTriggerSub; // user input for initialize,
    bool _initTrigger;                                                     // become true when /init topic published
};

BOOST_DLL_ALIAS(turtlesim::ALICATurtleWorldModel::init, WMInit)
BOOST_DLL_ALIAS(turtlesim::ALICATurtleWorldModel::del, WMDel)

} // namespace turtlesim
