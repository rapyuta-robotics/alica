#ifndef ALICA_TURTLE_WORLD_MODEL_WORLD_MODEL_HPP
#define ALICA_TURTLE_WORLD_MODEL_WORLD_MODEL_HPP

#include "turtle.hpp"
#include <engine/AlicaEngine.h>

#include <std_msgs/msg/empty.hpp>

#include <rclcpp/rclcpp.hpp>

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

} // namespace turtlesim

#endif /* ALICA_TURTLE_WORLD_MODEL_WORLD_MODEL_HPP */
