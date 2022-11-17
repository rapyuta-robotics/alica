#include "WaitBehaviour.h"
#include <thread>

namespace alica
{

WaitBehaviour::WaitBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
    std::cerr << "WaitBehaviour created" << std::endl;
}

void WaitBehaviour::run(void* msg)
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
} // namespace alica
