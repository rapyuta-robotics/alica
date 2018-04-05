#pragma once

#include "AlicaElement.h"
#include "engine/constraintmodul/SolverVariable.h"

#include <memory>
#include <string>

namespace alica {
class ModelFactory;
/**
 * A variable is constraint by conditions, feasible values can be queried using a ConstraintQuery.
 */
class Variable : public AlicaElement {
public:
    Variable();
    Variable(const std::shared_ptr<SolverVariable>& v);
    Variable(int64_t id, const std::string& type);
    virtual ~Variable();

    std::string toString() const;

    const std::string& getType() const {return _type;}
    std::shared_ptr<SolverVariable> getSolverVar() const {return _solverVar;}

    friend std::ostream& operator<<(std::ostream& os, const Variable& variable) {
        return os << variable.getName() << "(" << variable.getId() << ")";
    }

private:
    friend ModelFactory;
    void setType(const std::string& type);
    void setSolverVar(const std::shared_ptr<SolverVariable>& solverVar);
    std::shared_ptr<SolverVariable> _solverVar;
    std::string _type;

};

} /* namespace alica */
