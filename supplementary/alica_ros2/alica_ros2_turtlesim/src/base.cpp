#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicConstraintCreator.h>
#include <DynamicLoadingUtils.h>
#include <DynamicPlanCreator.h>
#include <DynamicTransitionConditionCreator.h>
#include <DynamicUtilityFunctionCreator.h>
#include <alica_ros2_turtlesim/base.hpp>
#include <alica_ros2_turtlesim/turtle_ros2_interfaces.hpp>
#include <boost/dll/import.hpp> // for import_alias
#include <constraintsolver/CGSolver.h>
#include <engine/AlicaContext.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_clock/AlicaRosTimer.h>
#include <ros2_communication/AlicaRosCommunication.h>
#include <ros2_logger/AlicaRosLogger.h>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>

namespace turtlesim
{

Base::Base(rclcpp::Node::SharedPtr nh, const std::string& name, const int agent_id, const std::string& roleset, const std::string& master_plan,
        const std::vector<std::string>& paths)
        : spinner(rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 1))
        , _nh(nh)
        , _name(name)
{
    // create world model
    spinner.add_node(nh);

    // Initialize Alica
    ac = std::make_unique<alica::AlicaContext>(AlicaContextParams(name, paths, roleset, master_plan, false, agent_id));

    ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
    ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
    ac->setLogger<alicaRosLogger::AlicaRosLogger>(agent_id);

    LockedBlackboardRW bb(ac->editGlobalBlackboard());
    bb.set<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle", std::make_shared<turtlesim::TurtleRos2Interfaces>(name));
    bb.set("spawned", false);
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
    spinner.spin_until_future_complete(result);
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
    spinner.spin_until_future_complete(result);
    RCLCPP_INFO(nh->get_logger(), "Spawned %s", name.c_str());
}

void Base::start()
{
    alica::AlicaCreators creators(std::make_unique<DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
            std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
            std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>());

    killMyTurtle(_name, _nh);
    spawnMyTurtle(_name, _nh);
    ac->init(std::move(creators));
    ac->addSolver<alica::reasoner::CGSolver>(ac->getConfig());
    spinner.spin();
}

Base::~Base()
{
    spinner.cancel();
    ac->terminate();
}

} // namespace turtlesim
