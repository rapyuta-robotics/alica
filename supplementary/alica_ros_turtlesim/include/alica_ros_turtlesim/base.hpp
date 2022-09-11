#ifndef IO_TURTLE_BASE_BASE_HPP
#define IO_TURTLE_BASE_BASE_HPP

#include <engine/AlicaEngine.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace turtlesim
{
class AlicaContext;
class Base
{
public:
    Base(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& name, const int agent_id, const std::string& roleset, const std::string& master_plan,
            const std::string& path);
    ~Base();
    void start();

private:
    ros::AsyncSpinner spinner;
    alica::AlicaContext* ac;
    void ALICATurtleWorldModelCallInit(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    void ALICATurtleWorldModelCallDel();
};

} // namespace turtlesim

#endif /* IO_TURTLE_BASE_BASE_HPP */
