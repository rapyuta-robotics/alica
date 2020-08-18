#include "alica/mockups/FailBehaviour.h"
namespace alica::mockups
{
FailBehaviour::FailBehaviour(const std::string& nameOfMockedBehaviour, uint32_t iterationsBeforeFail)
        : IMockUpBehaviour(nameOfMockedBehaviour)
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
