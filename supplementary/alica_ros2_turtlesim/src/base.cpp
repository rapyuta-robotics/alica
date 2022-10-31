#include <alica/BehaviourCreator.h>
#include <alica/ConditionCreator.h>
#include <alica/ConstraintCreator.h>
#include <alica/PlanCreator.h>
#include <alica/TransitionConditionCreator.h>
#include <alica/UtilityFunctionCreator.h>
#include <engine/AlicaContext.h>

#include <alica_ros2_turtlesim/base.hpp>
#include <constraintsolver/CGSolver.h>
#include <geometry_msgs/msg/twist.hpp>
#include <ros2_clock/AlicaRosTimer.h>
#include <ros2_communication/AlicaRosCommunication.h>
#include <ros2_logger/AlicaRosLogger.h>

#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>

namespace turtlesim
{

Base::Base(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr priv_nh, const std::string& name, const int agent_id, const std::string& roleset,
        const std::string& master_plan, const std::string& path)
        : spinner(rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 2))
        , _nh(nh)
        , _name(name)
{
    // create world model
    ALICATurtleWorldModel::init(nh, priv_nh);
    spinner.add_node(nh);
    spinner.add_node(priv_nh);
    // Initialize Alica
    ac = new alica::AlicaContext(AlicaContextParams(name, path + "/etc/", roleset, master_plan, false, agent_id));

    ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
    ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
    ac->setLogger<alicaRosLogger::AlicaRosLogger>(agent_id);

    spinThread = std::thread([this]() { spinner.spin(); });
}

void Base::killMyTurtle(const std::string& name, rclcpp::Node::SharedPtr nh)
{
    rclcpp::CallbackGroup::SharedPtr killCallback = nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client =
            nh->create_client<turtlesim::srv::Kill>("/kill", rmw_qos_profile_services_default, killCallback);
    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = name;
    auto result = kill_client->async_send_request(request);
    RCLCPP_INFO(nh->get_logger(), "Requested to Kill: %s", name.c_str());
    result.wait();
    RCLCPP_INFO(nh->get_logger(), "Request to Kill: %s complete", name.c_str());
}

void Base::spawnMyTurtle(const std::string& name, rclcpp::Node::SharedPtr nh)
{
    rclcpp::CallbackGroup::SharedPtr spawnCallback = nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client =
            nh->create_client<turtlesim::srv::Spawn>("/spawn", rmw_qos_profile_services_default, spawnCallback);
    auto request = std::make_shared<turtlesim::srv::Spawn_Request>();
    request->x = 1;
    request->y = 1;
    request->theta = 0;
    request->name = name;
    auto result = spawn_client->async_send_request(request);
    RCLCPP_INFO(nh->get_logger(), "Request to spawn %s", name.c_str());
    result.wait();
    RCLCPP_INFO(nh->get_logger(), "Spawned %s", name.c_str());
}

void Base::start()
{
    alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
            std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>(),
            std::make_unique<alica::TransitionConditionCreator>());

    killMyTurtle(_name, _nh);
    spawnMyTurtle(_name, _nh);
    ac->init(std::move(creators));
    ac->addSolver<alica::reasoner::CGSolver>();
}

Base::~Base()
{
    killMyTurtle(_name, _nh);
    spinner.cancel();
    spinThread.join();
    ac->terminate();
    delete ac;
    ALICATurtleWorldModel::del();
}

} // namespace turtlesim
