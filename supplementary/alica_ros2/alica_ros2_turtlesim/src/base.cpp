#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicConstraintCreator.h>
#include <DynamicLoadingUtils.h>
#include <DynamicPlanCreator.h>
#include <DynamicTransitionConditionCreator.h>
#include <DynamicUtilityFunctionCreator.h>
#include <alica_ros2_turtlesim/base.hpp>
#include <boost/dll/import.hpp> // for import_alias
#include <chrono>
#include <constraintsolver/CGSolver.h>
#include <engine/AlicaContext.h>
#include <future>
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
        const std::string& path, bool dedicatedEngineThread)
        : spinner(rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 1))
        , _nh(nh)
        , _name(name)
        , _dedicatedEngineThread(dedicatedEngineThread)
{
    // create world model
    spinner.add_node(nh);

    // Create the service clients before spinner runs, and keep them. Creating and destroying them on nh while spinner waits on it
    // crashes inside rmw_wait.
    turtle_service_group = nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    kill_client = nh->create_client<turtlesim::srv::Kill>("/kill", rmw_qos_profile_services_default, turtle_service_group);
    spawn_client = nh->create_client<turtlesim::srv::Spawn>("/spawn", rmw_qos_profile_services_default, turtle_service_group);

    // Initialize Alica
    ac = new alica::AlicaContext(AlicaContextParams(name, {path + "/etc/"}, roleset, master_plan, false, agent_id));

    ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
    ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();

    // The engine timer gets a callback group of its own, which spinner does not serve. With a dedicated thread, a
    // single-threaded executor on that thread serves the group and nothing else, so the engine may wait inside its timer
    // callback. Without one, the main loop serves the group through spinEngineOnce(), so the engine must not block it.
    engine_group = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    engine_executor.add_callback_group(engine_group, nh->get_node_base_interface());
    ac->setEngineTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(nh, engine_group, _dedicatedEngineThread);

    ac->setLogger<alicaRosLogger::AlicaRosLogger>(agent_id);

    spinThread = std::thread([this]() { spinner.spin(); });
}

namespace
{
// main() blocks SIGINT, so Ctrl+C cannot interrupt these waits: keep them bounded
constexpr std::chrono::seconds SERVICE_TIMEOUT{10};
} // namespace

void Base::killMyTurtle()
{
    // A request sent before the client has found the service is lost, and its response never arrives
    if (!kill_client->wait_for_service(SERVICE_TIMEOUT)) {
        RCLCPP_ERROR(_nh->get_logger(), "Service %s is not available, did not kill %s", kill_client->get_service_name(), _name.c_str());
        return;
    }
    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = _name;
    auto result = kill_client->async_send_request(request);
    RCLCPP_INFO(_nh->get_logger(), "Requested to Kill: %s", _name.c_str());
    if (result.wait_for(SERVICE_TIMEOUT) != std::future_status::ready) {
        RCLCPP_ERROR(_nh->get_logger(), "No response to the request to kill %s", _name.c_str());
        return;
    }
    RCLCPP_INFO(_nh->get_logger(), "Request to Kill: %s complete", _name.c_str());
}

void Base::spawnMyTurtle()
{
    if (!spawn_client->wait_for_service(SERVICE_TIMEOUT)) {
        RCLCPP_ERROR(_nh->get_logger(), "Service %s is not available, did not spawn %s", spawn_client->get_service_name(), _name.c_str());
        return;
    }
    auto request = std::make_shared<turtlesim::srv::Spawn_Request>();
    request->x = 1;
    request->y = 1;
    request->theta = 0;
    request->name = _name;
    auto result = spawn_client->async_send_request(request);
    RCLCPP_INFO(_nh->get_logger(), "Request to spawn %s", _name.c_str());
    if (result.wait_for(SERVICE_TIMEOUT) != std::future_status::ready) {
        RCLCPP_ERROR(_nh->get_logger(), "No response to the request to spawn %s", _name.c_str());
        return;
    }
    RCLCPP_INFO(_nh->get_logger(), "Spawned %s", _name.c_str());
}

void Base::start()
{
    alica::AlicaCreators creators(std::make_unique<DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
            std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
            std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>());

    killMyTurtle();
    spawnMyTurtle();
    if (_dedicatedEngineThread) {
        engineThread = std::thread([this]() { engine_executor.spin(); });
    }
    ac->init(std::move(creators));
    ac->addSolver<alica::reasoner::CGSolver>(ac->getConfig());
}

void Base::spinEngineOnce()
{
    if (!_dedicatedEngineThread) {
        engine_executor.spin_some();
    }
}

Base::~Base()
{
    killMyTurtle();
    // stop executors before terminating engine
    spinner.cancel();
    spinThread.join();
    if (engineThread.joinable()) {
        engine_executor.cancel();
        engineThread.join();
    }
    ac->terminate();
    delete ac;
}

} // namespace turtlesim
