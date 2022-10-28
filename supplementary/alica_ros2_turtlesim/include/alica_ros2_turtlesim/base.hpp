#ifndef IO_TURTLE_BASE_BASE_HPP
#define IO_TURTLE_BASE_BASE_HPP

#include "world_model.hpp"

#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#include <thread>

namespace turtlesim
{
class AlicaContext;
class Base
{
public:
    Base(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr priv_nh, const std::string& name, const int agent_id, const std::string& roleset,
            const std::string& master_plan, const std::string& path);
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
};

} // namespace turtlesim

#endif /* IO_TURTLE_BASE_BASE_HPP */
