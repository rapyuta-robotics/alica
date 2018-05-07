#pragma once

#include "alica/reasoner/DummyTerm.h"

#include <engine/constraintmodul/SolverVariable.h>

namespace alica {
namespace reasoner {

class DummyVariable : public DummyTerm, public alica::SolverVariable {
public:
    DummyVariable(long representingVariableID);
    virtual ~DummyVariable();
    long getID() const;

    static long ID_COUNTER;
    static std::string NO_VALUE;

private:
    long representingVariableID;
    long id;
};

} /* namespace reasoner */
} /* namespace alica */
