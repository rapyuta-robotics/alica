#pragma once

#include <engine/constraintmodul/SolverVariable.h>

#include "Term.h"

#include <string>
#include <memory>

namespace autodiff {
class Variable : public Term, public alica::SolverVariable {
public:
    double globalMin;
    double globalMax;

    Variable();

    int accept(std::shared_ptr<ITermVisitor> visitor);

    std::shared_ptr<Term> aggregateConstants();
    std::shared_ptr<Term> derivative(std::shared_ptr<Variable> v);

    std::string toString() override;

private:
    int ownId;
    static int var_id;
};
} /* namespace autodiff */
