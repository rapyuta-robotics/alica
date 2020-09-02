#include "alica/test/MockUpBehaviour.h"

namespace alica::test
{
MockUpBehaviour::MockUpBehaviour(const std::string& nameOfMockedBehaviour)
        : BasicBehaviour(nameOfMockedBehaviour)
        , _iterationsCounter(0)
{
}

uint32_t MockUpBehaviour::iterationsCounter() const
{
    return _iterationsCounter;
}

void MockUpBehaviour::incIterationsCounter()
{
    _iterationsCounter++;
}
} // namespace alica::mockups
