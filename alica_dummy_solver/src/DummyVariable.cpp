#include "alica/reasoner/DummyVariable.h"

namespace alica
{
namespace reasoner
{

std::string DummyVariable::NO_VALUE = "<NO-VALUE>";

DummyVariable::DummyVariable(int64_t representingVariableID)
    : alica::SolverVariable(representingVariableID)
{
}

} /* namespace reasoner */
} /* namespace alica */
