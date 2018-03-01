#include "DummyTerm.h"

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

void DummyTerm::setVariable(std::shared_ptr<alica::reasoner::DummyVariable> variable, std::string value)
{
    auto mapEntry = this->variableValueMap.find(variable.id);
    if (mapEntry != this->variableValueMap.end())
    {
        mapEntry->second = value;
    }
    else
    {
        this->variableValueMap.emplace(variable.id, value);
    }
}

std::string DummyTerm::getValue(std::shared_ptr<alica::reasoner::DummyVariable> variable)
{
    auto mapEntry = this->variableValueMap.find(variable.id);
    if (mapEntry != this->variableValueMap.end())
    {
        return mapEntry->second;
    }
    else
    {
        return "";
    }
}

} /* namespace reasoner */
} /* namespace alica */
