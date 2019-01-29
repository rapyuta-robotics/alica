#pragma once

#include "engine/BasicBehaviour.h"

namespace alica
{
class DomainBehaviour : public BasicBehaviour
{
public:
    DomainBehaviour(std::string name);
    virtual ~DomainBehaviour();
};
} /* namespace alica */
