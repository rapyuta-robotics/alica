#pragma once

#include "IMockUpBehaviour.h"

namespace alica::test
{
class FailBehaviour : public IMockUpBehaviour
{
    explicit FailBehaviour(uint32_t iterationsBeforeFail = 0);
    void run(void* msg) override;

private:
    uint32_t _iterationsBeforeFail;
};
} // namespace alica::mockups