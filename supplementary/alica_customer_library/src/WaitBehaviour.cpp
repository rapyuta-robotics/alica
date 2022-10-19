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
    int myTime = 1;

    // if (!getBlackboard()) {
    //     std::cerr << "Blackboard null" << std::endl;
    // }
    //  LockedBlackboardRW bb(*(getBlackboard()));
    /*
    int myTime=bb.get<int>("masterKey");
    std::cerr<<"Wait for:"<<myTime<<std::endl;
    */
    std::this_thread::sleep_for(std::chrono::seconds(myTime));
}
} // namespace alica
