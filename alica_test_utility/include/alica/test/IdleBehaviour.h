#pragma once

#include <engine/BasicBehaviour.h>

namespace alica
{
class BehaviourContext;
}

namespace alica::test
{
class IdleBehaviour : public BasicBehaviour
{
public:
    explicit IdleBehaviour(BehaviourContext& context)
            : BasicBehaviour(context){};
    void run() override;
};
} // namespace alica::test
