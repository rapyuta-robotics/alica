#pragma once

#include <engine/BasicBehaviour.h>

namespace alica::test
{
class IdleBehaviour : public BasicBehaviour
{
public:
    explicit IdleBehaviour()
            : BasicBehaviour("IdleBehaviour"){};
    void run(void* msg) override;
};
} // namespace alica::test