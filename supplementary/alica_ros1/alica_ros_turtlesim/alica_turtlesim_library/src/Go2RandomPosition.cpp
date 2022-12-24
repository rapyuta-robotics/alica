#include "Go2RandomPosition.h"
#include "world_model.hpp"
#include <memory>
#include <random>

namespace alica
{

Go2RandomPosition::Go2RandomPosition(BehaviourContext& context)
        : BasicBehaviour(context)
{
    std::cerr << "Debug:"
              << "Go2RandomPosition created" << std::endl;
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
    turtlesim::ALICATurtleWorldModel* wm = LockedBlackboardRW(getGlobalBlackboard()).get<std::shared_ptr<turtlesim::ALICATurtleWorldModel>>("worldmodel").get();

    wm->turtle.teleport(dist(engine), dist(engine));
    wm->setInit(false);
    setSuccess();
}
void Go2RandomPosition::initialiseParameters() {}

} /* namespace alica */
