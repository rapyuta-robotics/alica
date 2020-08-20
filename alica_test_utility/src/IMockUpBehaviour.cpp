#include "alica/test/IMockUpBehaviour.h"

namespace alica::test
{
IMockUpBehaviour::IMockUpBehaviour(const std::string& nameOfMockedBehaviour)
        : BasicBehaviour(nameOfMockedBehaviour)
        , _iterationsCounter(0)
{
}

uint32_t IMockUpBehaviour::iterationsCounter() const
{
    return _iterationsCounter;
}

void IMockUpBehaviour::incIterationsCounter()
{
    _iterationsCounter++;
}
} // namespace alica::mockups
