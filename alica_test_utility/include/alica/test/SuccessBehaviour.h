#pragma once

#include "IMockUpBehaviour.h"

namespace alica::test
{
class SuccessBehaviour : public IMockUpBehaviour
{
    explicit SuccessBehaviour(uint32_t iterationsBeforeSuccess = 0);
    void run(void* msg) override;
private:
    uint32_t _iterationsBeforeSuccess;
};
} // namespace alica::mockups