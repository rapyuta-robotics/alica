#include "alica/reasoner/SimpleVariable.h"

namespace alica
{
namespace reasoner
{

std::string SimpleVariable::NO_VALUE = "<NO-VALUE>";

SimpleVariable::SimpleVariable(int64_t representingVariableID)
    : alica::SolverVariable(representingVariableID)
{
}

} /* namespace reasoner */
} /* namespace alica */
