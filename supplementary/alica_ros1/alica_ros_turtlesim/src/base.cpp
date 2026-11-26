#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicConstraintCreator.h>
#include <DynamicLoadingUtils.h>
#include <DynamicPlanCreator.h>
#include <DynamicTransitionConditionCreator.h>
#include <DynamicUtilityFunctionCreator.h>
#include <alica_ros_turtlesim/base.hpp>
#include <alica_ros_turtlesim/turtle_ros1_interfaces.hpp>
#include <clock/AlicaROSClock.h>
#include <clock/AlicaRosTimer.h>
#include <communication/AlicaRosCommunication.h>
#include <constraintsolver/CGSolver.h>
#include <engine/AlicaContext.h>
#include <engine/logging/Logging.h>
#include <logger/AlicaRosLogger.h>
#include <ros/ros.h>

namespace turtlesim
{

Base::Base(ros::NodeHandle& nh, ros::NodeHandle& privNh, const std::string& name, const int agent_id, const std::string& roleset,
        const std::string& master_plan, const std::vector<std::string>& paths, std::optional<std::string> placeholderMapping, bool dedicatedEngineThread)
        : spinner(0)
        , _dedicatedEngineThread(dedicatedEngineThread)
{
    // Initialize Alica
    ac = std::make_unique<alica::AlicaContext>(AlicaContextParams(name, paths, roleset, master_plan, false, agent_id, placeholderMapping));

    ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>();
    ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();

    // The engine timer gets a queue of its own. With a dedicated thread, a single-threaded spinner serves that queue and nothing
    // else, so the engine may wait inside its timer callback. Without one, the main loop serves the queue through spinEngineOnce(),
    // so the engine must not block it.
    if (_dedicatedEngineThread) {
        engine_spinner.emplace(1, &engine_queue);
    }
    ac->setEngineTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(engine_queue, _dedicatedEngineThread);

    ac->setLogger<alicaRosLogger::AlicaRosLogger>();

    LockedBlackboardRW bb(ac->editGlobalBlackboard());
    bb.set<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle", std::make_shared<turtlesim::TurtleRos1Interfaces>(name));
    bb.set("spawned", false);
}

void Base::start()
{
    alica::AlicaCreators creators(std::make_unique<DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
            std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
            std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>());

    spinner.start(); // start spinners before initializing engine, but after setting context
    if (engine_spinner) {
        engine_spinner->start();
    }
    ac->init(std::move(creators), false);
    ac->addSolver<alica::reasoner::CGSolver>(ac->getConfig());
}

void Base::spinEngineOnce()
{
    if (!_dedicatedEngineThread) {
        engine_queue.callAvailable(ros::WallDuration(0));
    }
}

Base::~Base()
{
    // stop spinners before terminating engine
    spinner.stop();
    if (engine_spinner) {
        engine_spinner->stop();
    }
    ac->terminate();
}

} // namespace turtlesim
