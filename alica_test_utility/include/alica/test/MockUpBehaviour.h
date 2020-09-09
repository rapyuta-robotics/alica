#pragma once

#include <engine/BasicBehaviour.h>

namespace alica::test
{
class MockUpBehaviour : public alica::BasicBehaviour
{
public:
    explicit MockUpBehaviour(const std::string& nameOfMockedBehaviour);

protected:
    uint32_t iterationsCounter() const;
    void incIterationsCounter();

private:
    uint32_t _iterationsCounter;
};
} // namespace alica::mockups
