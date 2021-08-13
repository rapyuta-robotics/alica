#include "alica/test/IdleBehaviour.h"
#include <thread>
namespace alica::test
{
void IdleBehaviour::run(void* msg)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
} // namespace alica::test
