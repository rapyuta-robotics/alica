#pragma once

#include "alica/mockups/IMockUpBehaviour.h"

namespace alica::mockups
{
class FailBehaviour : public IMockUpBehaviour
{
    explicit FailBehaviour(const std::string& nameOfMockedBehaviour, uint32_t iterationsBeforeFail = 0);
    void run(void* msg) override;

private:
    uint32_t _iterationsBeforeFail;
};
} // namespace alica::mockups