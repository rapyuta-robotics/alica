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
               // create world model
}

Go2RandomPosition::~Go2RandomPosition() {}

void Go2RandomPosition::run(void* msg)
{
    std::cerr << "Debug:"
              << "11111111111" << std::endl;
    if (isSuccess()) {
        return;
    }
    std::cerr << "Debug:"
              << "2222222222" << std::endl;
    // code for generate random value between 0 and 10 which is default turtlesim area size
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    std::uniform_real_distribution<> dist(0, 10.0);
    std::cerr << "Debug:"
              << "33333333333" << std::endl;
    // teleport turtle to random place
    turtlesim::ALICATurtleWorldModel::get()->turtle.teleport(dist(engine), dist(engine));
        std::cerr << "Debug:"
              << "444444444" << std::endl;
    turtlesim::ALICATurtleWorldModel::get()->setInit(false);
    setSuccess();
    std::cerr << "Debug:"
              << "55555555555" << std::endl;
}
void Go2RandomPosition::initialiseParameters() {}

} /* namespace alica */
