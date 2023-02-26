#include "Go2RandomPosition.h"
#include <memory>

#include "world_model.hpp"
#include <random>

namespace turtlesim
{

Go2RandomPosition::Go2RandomPosition(alica::BehaviourContext& context)
        : BasicBehaviour(context)
{
}
Go2RandomPosition::~Go2RandomPosition() {}
void Go2RandomPosition::run()
{
    if (isSuccess()) {
        return;
    }

    // code for generate random value between 0 and 10 which is default turtlesim area size
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::uniform_real_distribution<> dist(0, 10.0);

    // teleport turtle to random place
    turtlesim::ALICATurtleWorldModel::get()->turtle->teleport(dist(engine), dist(engine));
    turtlesim::ALICATurtleWorldModel::get()->setInit(false);
    setSuccess();
}
void Go2RandomPosition::initialiseParameters() {}
std::unique_ptr<Go2RandomPosition> Go2RandomPosition::create(alica::BehaviourContext& context)
{
    return std::make_unique<Go2RandomPosition>(context);
}

} /* namespace turtlesim */
