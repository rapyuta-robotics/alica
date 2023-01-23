#include "Teleport2RandomPosition.h"
#include "world_model.hpp"

#include <memory>
#include <random>

namespace turtlesim
{

Teleport2RandomPosition::Teleport2RandomPosition(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

void Teleport2RandomPosition::run()
{
    if (isSuccess()) {
        return;
    }
    // code for generate random value between 0 and 10 which is default turtlesim area size
    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());
    std::uniform_real_distribution<> dist(0, 10.0);
    // teleport turtle to random place
    std::shared_ptr<turtlesim::ALICATurtleWorldModel> wm =
            alica::LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::ALICATurtleWorldModel>>("worldmodel");

    wm->turtle.teleport(dist(engine), dist(engine));
    setSuccess();
}

std::unique_ptr<Teleport2RandomPosition> Teleport2RandomPosition::create(alica::BehaviourContext& context)
{
    return std::make_unique<Teleport2RandomPosition>(context);
}

} /* namespace turtlesim */
