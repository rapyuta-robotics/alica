#pragma once

#include "IMockUpBehaviour.h"

namespace alica::test
{
class IdleBehaviour : public IMockUpBehaviour
{
public:
    explicit IdleBehaviour()
            : IMockUpBehaviour("IdleBehaviour"){};
    void run(void* msg) override;
};
} // namespace alica::mockups