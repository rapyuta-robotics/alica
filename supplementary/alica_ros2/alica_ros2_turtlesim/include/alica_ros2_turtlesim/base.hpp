#pragma once

#include <memory>
#include <thread>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>

namespace alica
{
class AlicaContext;
}

namespace turtlesim
{

class Base
{
public:
    Base(rclcpp::Node::SharedPtr nh, const std::string& name, const int agent_id, const std::string& roleset, const std::string& master_plan,
            const std::string& path);
    ~Base();
    void start();
    void killMyTurtle(const std::string& name, rclcpp::Node::SharedPtr);
    void spawnMyTurtle(const std::string& name, rclcpp::Node::SharedPtr);

private:
    rclcpp::executors::MultiThreadedExecutor spinner;
    alica::AlicaContext* ac;
    std::thread spinThread;
    std::string _name;
    rclcpp::Node::SharedPtr _nh;

    void ALICATurtleWorldModelCallInit();
    void ALICATurtleWorldModelCallDel();
};

} // namespace turtlesim
