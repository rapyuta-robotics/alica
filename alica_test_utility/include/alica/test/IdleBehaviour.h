#pragma once

#include "MockUpBehaviour.h"

namespace alica::test
{
class IdleBehaviour : public MockUpBehaviour
{
public:
    explicit IdleBehaviour()
            : MockUpBehaviour("IdleBehaviour"){};
    void run(void* msg) override;
};
} // namespace alica::mockups