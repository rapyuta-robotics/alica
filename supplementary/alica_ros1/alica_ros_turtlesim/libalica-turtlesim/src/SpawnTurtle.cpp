#include "SpawnTurtle.h"

#include <ros/ros.h>

#include <memory>
#include <random>

#include "turtle_interfaces.hpp"
#include <engine/logging/Logging.h>

namespace turtlesim
{

SpawnTurtle::SpawnTurtle(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

void SpawnTurtle::initialiseParameters()
{
    // When we spawn the turtle, instantiate turtle interface into the global blackboard
    // These interface will be retrieved from global blackboard and used by other plans using the turtle

    // TODO: Detect and fail if this behavior is called twice
    alica::LockedBlackboardRW g_bb(*getGlobalBlackboard());
    auto name = g_bb.get<std::string>("agentName");
    if (g_bb.hasValue("turtle")) {
        Logging::logWarn("SpawnTurtle") << name << " was already spawned";
        setFailure();
        return;
    }
    g_bb.set<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle", std::make_shared<turtlesim::TurtleInterfaces>(name));

    auto turtleInterfaces = g_bb.get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");

    if (turtleInterfaces->spawn()) {
        Logging::logInfo("SpawnTurtle") << "SpawnTurtle", name << " was spawned";
        setSuccess();
    } else {
        Logging::logWarn("SpawnTurtle") << "Failed to spawn " << name << ".  Succeeding anyway";
        setSuccess();
    }
}

std::unique_ptr<SpawnTurtle> SpawnTurtle::create(alica::BehaviourContext& context)
{
    return std::make_unique<SpawnTurtle>(context);
}

} /* namespace turtlesim */
