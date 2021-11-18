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
            : BasicBehaviour("IdleBehaviour", wm){};
    void run(void* msg) override;
};
} // namespace alica::mockups