#include "Go2RandomPosition.h"
#include "engine/RunningPlan.h"
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

void Go2RandomPosition::run(void* msg)
{
    if (isSuccess()) {
        return;
    }
    // code for generate random value between 0 and 10 which is default turtlesim area size
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::uniform_real_distribution<> dist(0, 10.0);

    // teleport turtle to random place
    LockedBlackboardRW bb(*(getBlackboard()));
    if (!bb.hasValue("turtlesim::worldmodel")) {
        std::cerr << "Errro:Blackboard for Go2RandomPosition not found" << std::endl;
        return;
    }
    turtlesim::ALICATurtleWorldModel* wm = bb.get<turtlesim::ALICATurtleWorldModel*>("turtlesim::worldmodel");
    wm->turtle.teleport(dist(engine), dist(engine));
    wm->setInit(false);
    setSuccess();
}
void Go2RandomPosition::initialiseParameters() {}

} /* namespace alica */
