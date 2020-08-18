#pragma once

#include "alica/mockups/IMockUpBehaviour.h"

namespace alica::mockups
{
class IdleBehaviour : public IMockUpBehaviour
{
public:
    explicit IdleBehaviour(const std::string& nameOfMockedBehaviour)
            : IMockUpBehaviour(nameOfMockedBehaviour){};
    void run(void* msg) override;
};
} // namespace alica::mockups