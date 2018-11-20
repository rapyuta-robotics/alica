#pragma once

#include "alica/reasoner/DummyTerm.h"

#include <alica_solver_interface/SolverVariable.h>

namespace alica
{
namespace reasoner
{

class DummyVariable : public DummyTerm, public alica::SolverVariable
{
  public:
    DummyVariable(int64_t representingVariableID);
    static std::string NO_VALUE;
};

} /* namespace reasoner */
} /* namespace alica */
