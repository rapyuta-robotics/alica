#pragma once

#include <engine/BasicBehaviour.h>
#include <string>

namespace alica
{
class DomainBehaviour : public BasicBehaviour
{
public:
    DomainBehaviour(BehaviourContext& context);
    virtual ~DomainBehaviour();

};
} /* namespace alica */
