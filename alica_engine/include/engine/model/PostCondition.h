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
    std::string toString() const override;
};

} // namespace alica
