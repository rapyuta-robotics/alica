#include "alica/reasoner/DummyTerm.h"
#include "alica/reasoner/DummyVariable.h"

namespace alica
{
namespace reasoner
{

DummyTerm::DummyTerm()
{
}

DummyTerm::~DummyTerm()
{
}

void DummyTerm::setVariable(std::shared_ptr<DummyVariable> variable, std::string value)
{
    auto mapEntry = this->variableValueMap.find(variable->getID());
    if (mapEntry != this->variableValueMap.end())
    {
        mapEntry->second = value;
    }
    else
    {
        this->variableValueMap.emplace(variable->getID(), value);
    }
}

std::string DummyTerm::getValue(std::shared_ptr<DummyVariable> variable)
{
    auto mapEntry = this->variableValueMap.find(variable->getID());
    if (mapEntry != this->variableValueMap.end())
    {
        return mapEntry->second;
    }
    else
    {
        return DummyVariable::NO_VALUE;
    }
}

} /* namespace reasoner */
} /* namespace alica */
