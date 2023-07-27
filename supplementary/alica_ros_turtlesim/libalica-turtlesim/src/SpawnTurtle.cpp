#include "SpawnTurtle.h"

#include <memory>
#include <random>

#include <alica_turtlesim/turtle_interfaces.hpp>
#include <engine/logging/Logging.h>

using Logging = alica::Logging;

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
    if (g_bb.hasValue("spawned") && g_bb.get<bool>("spawned")) {
        setFailure();
        return;
    }
    auto turtleInterfaces = g_bb.get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");

    if (turtleInterfaces->spawn()) {
        g_bb.set("spawned", true);
        setSuccess();
    } else {
        setSuccess();
    }
}

std::unique_ptr<SpawnTurtle> SpawnTurtle::create(alica::BehaviourContext& context)
{
    return std::make_unique<SpawnTurtle>(context);
}

} /* namespace turtlesim */
