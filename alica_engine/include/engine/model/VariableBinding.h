#pragma once

#include <string>

#include "AlicaElement.h"

namespace alica
{
class Variable;
class AbstractPlan;
class VariableBindingFactory;

class VariableBinding : public AlicaElement
{
public:
    VariableBinding();
    virtual ~VariableBinding();

    std::string toString(std::string indent = "") const override;

    const AbstractPlan* getSubPlan() const { return _subPlan; }
    const Variable* getVar() const { return _var; }
    const Variable* getSubVar() const { return _subVar; }

protected:
    friend ModelFactory;
    friend VariableBindingFactory;

    void setSubPlan(const AbstractPlan* subPlan);
    void setSubVar(const Variable* subVar);
    void setVar(const Variable* var);

    const Variable* _var;
    const Variable* _subVar;
    const AbstractPlan* _subPlan;
};

} // namespace alica

