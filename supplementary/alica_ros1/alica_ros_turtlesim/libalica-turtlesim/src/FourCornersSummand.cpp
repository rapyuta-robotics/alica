#include "FourCornersSummand.h"
#include "engine/blackboard/Blackboard.h"
#include "engine/planselector/IAssignment.h"
#include "turtle_interfaces.hpp"
#include <fstream>
#include <iostream>

namespace turtlesim
{

FourCornersSummand::FourCornersSummand(double weight)
        : USummand(weight)
{
}

FourCornersSummand::~FourCornersSummand() {}

/*
    The goal of this summand is to priotize agent to pick up corner task which is closest to it.
    We evaluate current assignment to priotize assignment if current agent is assigned task which is to it's closest corner
*/
alica::UtilityInterval FourCornersSummand::eval(alica::IAssignment ass, const alica::Assignment* oldAss, const alica::Blackboard* globalBlackboard) const
{
    // co-ordinates of each corner
    std::vector<std::pair<double, double>> coOrdinateInfo{{1.5, 1.5}, {1.5, 8.5}, {8.5, 1.5}, {8.5, 8.5}};
    auto turtle = alica::LockedBlackboardRO(*globalBlackboard).get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");
    ros::Time startTime = ros::Time::now();
    // blocking wait until turtle pose are valid
    while (!turtle->getCurrentPose() && ros::Time::now() - startTime < ros::Duration(5)) {
    };
    if (!turtle->getCurrentPose()) {
        alica::Logging::logInfo("FourCornersSummand") << "Turtle pose not valid";
        return alica::UtilityInterval(0, 0);
    }
    const auto x = turtle->getCurrentPose()->x;
    const auto y = turtle->getCurrentPose()->y;
    double bestDistane = std::numeric_limits<double>::max();
    double currentAgentDistance = std::numeric_limits<double>::max();
    std::pair<double, double> currentCorner, bestCorner;
    for (auto entryPoint : _relevantEntryPoints) {
        auto distanceSqr = std::pow(x - coOrdinateInfo[entryPoint->getIndex()].first, 2) + std::pow(y - coOrdinateInfo[entryPoint->getIndex()].second, 2);
        for (alica::AgentId agent : ass.getAgentsWorking(entryPoint)) {
            if (agent == turtle->id()) {
                // track distance of current agent to the corner task which is being assigned in this assignment
                currentAgentDistance = distanceSqr;
                currentCorner = coOrdinateInfo[entryPoint->getIndex()];
            }
        }
        // track the distance of the best corner task for the current agent
        if (distanceSqr < bestDistane) {
            bestDistane = distanceSqr;
            bestCorner = coOrdinateInfo[entryPoint->getIndex()];
        }
    }
    // if there is a corner task which is better than agent's current tasks in this assginment then we deprioritize this assginment
    if (bestDistane < currentAgentDistance) {
        return alica::UtilityInterval(0, 0);
    }
    // we priotize this assignment if current agent is assigned better corner task
    return alica::UtilityInterval(0.5, 1);
}

} /* namespace turtlesim */
