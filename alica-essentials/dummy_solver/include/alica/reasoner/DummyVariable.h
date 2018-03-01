#pragma once

#include "alica/reasoner/DummyTerm.h"

#include <engine/constraintmodul/SolverVariable.h>

namespace alica
{
namespace reasoner
{

class DummyVariable : public DummyTerm, public alica::SolverVariable
{
  public:
    DummyVariable(long representingVariableID);
    virtual ~DummyVariable();

  private:
    long representingVariableID;
};

} /* namespace reasoner */
} /* namespace alica */
