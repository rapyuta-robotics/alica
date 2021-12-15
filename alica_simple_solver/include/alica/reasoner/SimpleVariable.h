#pragma once

#include "alica/reasoner/SimpleTerm.h"

#include <alica_solver_interface/SolverVariable.h>

namespace alica
{
namespace reasoner
{

class SimpleVariable : public SimpleTerm, public alica::SolverVariable
{
public:
    SimpleVariable(int64_t representingVariableID);
    static std::string NO_VALUE;
};

} /* namespace reasoner */
} /* namespace alica */
