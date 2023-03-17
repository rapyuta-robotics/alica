#pragma once

#include "AlicaElement.h"

#include <iostream>
#include <memory>
#include <string>

namespace alica
{
class VariableFactory;
/**
 * A variable is constraint by conditions, feasible values can be queried using a ConstraintQuery.
 */
class Variable : public AlicaElement
{
public:
    Variable(const std::string& type);
    Variable(int64_t id, const std::string& name, const std::string& type);

    std::string toString(std::string indent = "") const override;
    const std::string& getType() const { return _type; }

    friend std::ostream& operator<<(std::ostream& os, const Variable& variable) { return os << variable.getName() << "(" << variable.getId() << ")"; }

private:
    std::string _type;
};

} /* namespace alica */
