#pragma once

#include <string>

#include "Condition.h"

namespace alica
{

class PostCondition : public Condition
{
public:
    PostCondition(int64_t id = 0);
    virtual ~PostCondition();
    std::string toString() const override;
};

} // namespace alica
