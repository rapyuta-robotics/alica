#pragma once

#include <string>

#include "Condition.h"

namespace alica
{

class PostCondition : public Condition
{
public:
    PostCondition();
    virtual ~PostCondition();
    std::string toString(std::string indent = "") const override;
};

} // namespace alica
