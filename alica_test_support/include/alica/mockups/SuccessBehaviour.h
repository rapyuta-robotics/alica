#pragma once

#include "alica/mockups/IMockUpBehaviour.h"

namespace alica::mockups
{
class SuccessBehaviour : public IMockUpBehaviour
{
    explicit SuccessBehaviour(const std::string& nameOfMockedBehaviour, uint32_t iterationsBeforeSuccess = 0);
    void run(void* msg) override;
private:
    uint32_t _iterationsBeforeSuccess;
};
} // namespace alica::mockups