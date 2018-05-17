#pragma once

#include "AlicaElement.h"
#include "alica_solver_interface/SolverVariable.h"

#include <iostream>
#include <memory>
#include <string>

namespace alica
{
class ModelFactory;
/**
 * A variable is constraint by conditions, feasible values can be queried using a ConstraintQuery.
 */
class Variable : public AlicaElement
{
  public:
    Variable();
    Variable(int64_t id, const std::string& name, const std::string& type);
    virtual ~Variable();

    std::string toString() const override;

    const std::string& getType() const { return _type; }
    // std::shared_ptr<SolverVariable> getSolverVar() const { return _solverVar; }

    friend std::ostream& operator<<(std::ostream& os, const Variable& variable) { return os << variable.getName() << "(" << variable.getId() << ")"; }
    // void setSolverVar(const std::shared_ptr<SolverVariable>& solverVar) const;

  private:
    friend ModelFactory;
    void setType(const std::string& type);
    // mutable std::shared_ptr<SolverVariable> _solverVar;  // TODO: move out of here
    std::string _type;
};

} /* namespace alica */
