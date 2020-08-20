#include "alica/test/SuccessBehaviour.h"
namespace alica::test
{

SuccessBehaviour::SuccessBehaviour(uint32_t iterationsBeforeSuccess)
        : IMockUpBehaviour("SuccesBehaviour")
        , _iterationsBeforeSuccess(iterationsBeforeSuccess){};

void SuccessBehaviour::run(void* msg)
{
    if (iterationsCounter() >= _iterationsBeforeSuccess && !this->isSuccess()) {
        this->setSuccess();
    }
    incIterationsCounter();
}
} // namespace alica::mockups
