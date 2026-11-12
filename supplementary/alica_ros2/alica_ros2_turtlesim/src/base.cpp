#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicConstraintCreator.h>
#include <DynamicLoadingUtils.h>
#include <DynamicPlanCreator.h>
#include <DynamicTransitionConditionCreator.h>
#include <DynamicUtilityFunctionCreator.h>
#include <alica_ros2_turtlesim/base.hpp>
#include <boost/dll/import.hpp> // for import_alias
#include <clock/AlicaRosTimer.h>
#include <communication/AlicaRosCommunication.h>
#include <constraintsolver/CGSolver.h>
#include <engine/AlicaContext.h>
#include <geometry_msgs/msg/twist.hpp>
#include <logger/AlicaRosLogger.h>
#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim_msgs/srv/kill.hpp>
#include <turtlesim_msgs/srv/spawn.hpp>

namespace turtlesim
{

Base::Base(rclcpp::Node::SharedPtr nh, const std::string& name, const int agent_id, const std::string& roleset, const std::string& master_plan,
        const std::string& path)
        : spinner(rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 1))
        , _nh(nh)
        , _name(name)
{
    // create world model
    spinner.add_node(nh);

    // Initialize Alica
    ac = new alica::AlicaContext(AlicaContextParams(name, {path + "/etc/"}, roleset, master_plan, false, agent_id));

    ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
    ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
    ac->setLogger<alicaRosLogger::AlicaRosLogger>(agent_id);

    spinThread = std::thread([this]() { spinner.spin(); });
}

void Base::killMyTurtle(const std::string& name, rclcpp::Node::SharedPtr nh)
{
    rclcpp::CallbackGroup::SharedPtr killCallback = nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::Client<turtlesim_msgs::srv::Kill>::SharedPtr kill_client =
            nh->create_client<turtlesim_msgs::srv::Kill>("/kill", rclcpp::ServicesQoS(), killCallback);
    auto request = std::make_shared<turtlesim_msgs::srv::Kill::Request>();
    request->name = name;
    // A request that is sent before the service is discovered gets no response
    kill_client->wait_for_service();
    auto result = kill_client->async_send_request(request);
    RCLCPP_INFO(nh->get_logger(), "Requested to Kill: %s", name.c_str());
    result.wait();
    RCLCPP_INFO(nh->get_logger(), "Request to Kill: %s complete", name.c_str());
}

void Base::spawnMyTurtle(const std::string& name, rclcpp::Node::SharedPtr nh)
{
    rclcpp::CallbackGroup::SharedPtr spawnCallback = nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::Client<turtlesim_msgs::srv::Spawn>::SharedPtr spawn_client =
            nh->create_client<turtlesim_msgs::srv::Spawn>("/spawn", rclcpp::ServicesQoS(), spawnCallback);
    auto request = std::make_shared<turtlesim_msgs::srv::Spawn_Request>();
    request->x = 1;
    request->y = 1;
    request->theta = 0;
    request->name = name;
    spawn_client->wait_for_service();
    auto result = spawn_client->async_send_request(request);
    RCLCPP_INFO(nh->get_logger(), "Request to spawn %s", name.c_str());
    result.wait();
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
}

Base::~Base()
{
    killMyTurtle(_name, _nh);
    spinner.cancel();
    spinThread.join();
    ac->terminate();
    delete ac;
}

} // namespace turtlesim
