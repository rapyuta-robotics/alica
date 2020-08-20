#include "alica/test/FailBehaviour.h"
namespace alica::test
{
FailBehaviour::FailBehaviour( uint32_t iterationsBeforeFail)
        : IMockUpBehaviour("FailBehaviour")
        , _iterationsBeforeFail(iterationsBeforeFail)
{
}
void FailBehaviour::run(void* msg)
{
    if (iterationsCounter() >= _iterationsBeforeFail && !this->isFailure()) {
        this->setFailure();
    }
    incIterationsCounter();
}
} // namespace alica::mockups
