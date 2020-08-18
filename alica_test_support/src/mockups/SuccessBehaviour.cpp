#include "alica/mockups/SuccessBehaviour.h"
namespace alica ::mockups
{

SuccessBehaviour::SuccessBehaviour(const std::string& nameOfMockedBehaviour, uint32_t iterationsBeforeSuccess)
        : IMockUpBehaviour(nameOfMockedBehaviour)
        , _iterationsBeforeSuccess(iterationsBeforeSuccess){};

void SuccessBehaviour::run(void* msg)
{
    if (iterationsCounter() >= _iterationsBeforeSuccess && !this->isSuccess()) {
        this->setSuccess();
    }
    incIterationsCounter();
}
} // namespace alica::mockups
