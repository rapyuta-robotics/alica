#pragma once

#include "SolverVariable.h"
namespace alica
{
class SolverContext
{
public:
    virtual ~SolverContext(){};
    virtual void clear() = 0;
};
} // namespace alica
