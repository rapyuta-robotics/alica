#ifndef ALICA_TURTLE_WORLD_MODEL_WORLD_MODEL_HPP
#define ALICA_TURTLE_WORLD_MODEL_WORLD_MODEL_HPP

#include "alica_ros2_turtlesim/turtle.hpp"
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
    static ALICATurtleWorldModel* get();                                                       // return instance
    static void init(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> priv_nh); // create instance
    static void del();
    bool getInit() const { return _initTrigger; };
    void setInit(const bool input) { _initTrigger = input; };

    static ALICATurtleWorldModel* instance;
    ALICATurtle turtle;

private:
    ALICATurtleWorldModel(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> priv_nh);
    ~ALICATurtleWorldModel();
    void initTriggerSubCallback(const std_msgs::msg::Empty& msg);                // callback of /init
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> _initTriggerSub; // user input for initialize,
    bool _initTrigger;                                                           // become true when /init topic published
};

} // namespace turtlesim

#endif /* ALICA_TURTLE_WORLD_MODEL_WORLD_MODEL_HPP */
