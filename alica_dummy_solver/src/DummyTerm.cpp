#include "alica/reasoner/DummyTerm.h"
#include "alica/reasoner/DummyVariable.h"

namespace alica
{
namespace reasoner
{

DummyTerm::DummyTerm() {}

DummyTerm::~DummyTerm() {}

void DummyTerm::setVariable(DummyVariable* variable, const std::string& value)
{
    _variableValueMap[variable->getId()] = value;
}

const std::string& DummyTerm::getValue(DummyVariable* variable) const
{
    const auto mapEntry = _variableValueMap.find(variable->getId());
    if (mapEntry != _variableValueMap.end()) {
        return mapEntry->second;
    } else {
        return DummyVariable::NO_VALUE;
    }
}

const std::string* DummyTerm::tryGetValue(int64_t id) const
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
