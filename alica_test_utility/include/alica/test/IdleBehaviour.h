#pragma once

#include <engine/BasicBehaviour.h>

namespace alica
{
    class IAlicaWorldModel;
}

namespace alica::test
{
class IdleBehaviour : public BasicBehaviour
{
public:
    explicit IdleBehaviour(alica::IAlicaWorldModel* wm)
            : BasicBehaviour(wm, "IdleBehaviour"){};
    void run(void* msg) override;
};
} // namespace alica::mockups