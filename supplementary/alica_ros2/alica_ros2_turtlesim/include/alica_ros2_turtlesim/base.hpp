#pragma once

#include <memory>
#include <thread>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>

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

private:
    void killMyTurtle();
    void spawnMyTurtle();

    rclcpp::executors::MultiThreadedExecutor spinner;
    rclcpp::CallbackGroup::SharedPtr turtle_service_group;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client;
    alica::AlicaContext* ac;
    std::thread spinThread;
    std::string _name;
    rclcpp::Node::SharedPtr _nh;
};

} // namespace turtlesim
