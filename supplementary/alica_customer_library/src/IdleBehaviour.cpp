#include "alica/test/IdleBehaviour.h"
#include <thread>
namespace alica
{

IdleBehaviour::IdleBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
    std::cerr << "IdleBehaviour created" << std::endl;
}

void IdleBehaviour::run(void* msg)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
} // namespace alica
