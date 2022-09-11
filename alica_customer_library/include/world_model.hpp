#ifndef ALICA_TURTLE_WORLD_MODEL_WORLD_MODEL_HPP
#define ALICA_TURTLE_WORLD_MODEL_WORLD_MODEL_HPP

#include "turtle.hpp"
#include <boost/dll/alias.hpp>
#include <engine/AlicaEngine.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

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
    static ALICATurtleWorldModel* get();                             // return instance
    static void init(ros::NodeHandle& nh, ros::NodeHandle& priv_nh); // create instance
    static void del();
    bool getInit() const { return _initTrigger; };
    void setInit(const bool input) { _initTrigger = input; };

    static ALICATurtleWorldModel* instance;
    ALICATurtle turtle;

private:
    ALICATurtleWorldModel(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    ~ALICATurtleWorldModel();
    void initTriggerSubCallback(const std_msgs::EmptyConstPtr& msg); // callback of /init
    ros::Subscriber _initTriggerSub;                                 // user input for initialize,
    bool _initTrigger;                                               // become true when /init topic published
};

BOOST_DLL_ALIAS(turtlesim::ALICATurtleWorldModel::init, ALICATurtleWorldModelInit)
BOOST_DLL_ALIAS(turtlesim::ALICATurtleWorldModel::del, ALICATurtleWorldModelDel)

} // namespace turtlesim

#endif /* ALICA_TURTLE_WORLD_MODEL_WORLD_MODEL_HPP */