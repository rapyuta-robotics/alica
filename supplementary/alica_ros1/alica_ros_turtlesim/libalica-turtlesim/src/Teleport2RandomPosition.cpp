#include "Teleport2RandomPosition.h"
#include "turtle.hpp"

#include <memory>
#include <random>

namespace turtlesim
{

Teleport2RandomPosition::Teleport2RandomPosition(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

void Teleport2RandomPosition::initialiseParameters()
{
    // code for generate random value between 0 and 10 which is default turtlesim area size
    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());
    std::uniform_real_distribution<> dist(0, 10.0);
    // teleport turtle to random place
    auto turtle = alica::LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::ALICATurtle>>("turtle");

    turtle->teleport(dist(engine), dist(engine));
    setSuccess();
}

void Teleport2RandomPosition::run() {}

std::unique_ptr<Teleport2RandomPosition> Teleport2RandomPosition::create(alica::BehaviourContext& context)
{
    return std::make_unique<Teleport2RandomPosition>(context);
}

} /* namespace turtlesim */