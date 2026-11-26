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
    /**
     * dedicatedEngineThread: true runs the ALICA engine on a thread of its own. False leaves it to the caller's loop,
     * which must call spinEngineOnce() regularly.
     */
    Base(rclcpp::Node::SharedPtr nh, const std::string& name, const int agent_id, const std::string& roleset, const std::string& master_plan,
            const std::string& path, bool dedicatedEngineThread = true);
    ~Base();
    void start();
    // Runs the engine timer callbacks that are due. Only needed without a dedicated engine thread.
    void spinEngineOnce();

private:
    void killMyTurtle();
    void spawnMyTurtle();

    rclcpp::executors::MultiThreadedExecutor spinner;
    rclcpp::CallbackGroup::SharedPtr turtle_service_group;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client;
    // Serves engine_group only, which holds the engine timer
    rclcpp::executors::SingleThreadedExecutor engine_executor;
    rclcpp::CallbackGroup::SharedPtr engine_group;
    alica::AlicaContext* ac;
    std::thread spinThread;
    std::thread engineThread;
    std::string _name;
    rclcpp::Node::SharedPtr _nh;
    const bool _dedicatedEngineThread;
};

} // namespace turtlesim
