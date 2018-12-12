#include "alica/reasoner/SimpleTerm.h"
#include "alica/reasoner/SimpleVariable.h"

namespace alica
{
namespace reasoner
{

SimpleTerm::SimpleTerm() {}

SimpleTerm::~SimpleTerm() {}

void SimpleTerm::setVariable(SimpleVariable* variable, const std::string& value)
{
    _variableValueMap[variable->getId()] = value;
}

const std::string& SimpleTerm::getValue(SimpleVariable* variable) const
{
    const auto mapEntry = _variableValueMap.find(variable->getId());
    if (mapEntry != _variableValueMap.end()) {
        return mapEntry->second;
    } else {
        return SimpleVariable::NO_VALUE;
    }
}

const std::string* SimpleTerm::tryGetValue(int64_t id) const
{
    const auto mapEntry = _variableValueMap.find(id);
    if (mapEntry != _variableValueMap.end()) {
        return &mapEntry->second;
    } else {
        return nullptr;
    }
}

} /* namespace reasoner */
} /* namespace alica */
