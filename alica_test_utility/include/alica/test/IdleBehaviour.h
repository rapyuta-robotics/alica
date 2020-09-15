#pragma once

#include "SuccessOrFailBehaviour.hpp"

namespace alica::test
{
class IdleBehaviour : public BasicBehaviour
{
public:
    explicit IdleBehaviour()
            : BasicBehaviour("IdleBehaviour"){};
    void run(void* msg) override;
};
} // namespace alica::mockups