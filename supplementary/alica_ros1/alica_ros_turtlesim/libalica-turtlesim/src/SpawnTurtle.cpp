#include "SpawnTurtle.h"

#include <ros/ros.h>
#include <turtlesim/Spawn.h>

#include <memory>
#include <random>

namespace turtlesim
{

SpawnTurtle::SpawnTurtle(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

void SpawnTurtle::run()
{
    ros::ServiceClient spawnSrvClient = ros::NodeHandle("~").serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawnSrv;
    spawnSrv.request.x = 1;
    spawnSrv.request.y = 1;
    spawnSrv.request.theta = 0;
    spawnSrv.request.name = alica::LockedBlackboardRO(*getGlobalBlackboard()).get<std::string>("agentName");
    if (spawnSrvClient.waitForExistence() && spawnSrvClient.call(spawnSrv)) {
        ROS_INFO_STREAM_NAMED(__func__, spawnSrv.request.name << " was spawned");
    } else {
        ROS_ERROR_STREAM_NAMED(__func__, "Failed to spawn " << spawnSrv.request.name);
        setFailure();
    }
    setSuccess();
}

std::unique_ptr<SpawnTurtle> SpawnTurtle::create(alica::BehaviourContext& context)
{
    return std::make_unique<SpawnTurtle>(context);
}

} /* namespace turtlesim */
