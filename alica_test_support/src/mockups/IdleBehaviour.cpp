#include "alica/mockups/IdleBehaviour.h"
#include <thread>
namespace alica::mockups
{
void IdleBehaviour::run(void* msg)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
} // namespace alica::mockups
