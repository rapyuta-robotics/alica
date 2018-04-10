/*
 * Variable.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: psp
 */

#include "Variable.h"

#include "TermBuilder.h"

#include <limits>

namespace autodiff {
int Variable::var_id = 0;

Variable::Variable() {
    globalMin = -numeric_limits<double>::infinity();
    globalMax = numeric_limits<double>::infinity();
    ownId = var_id++;
}

int Variable::accept(shared_ptr<ITermVisitor> visitor) {
    shared_ptr<Variable> thisCasted = dynamic_pointer_cast<Variable>(shared_from_this());
    return visitor->visit(thisCasted);
}

std::shared_ptr<Term> Variable::aggregateConstants() {
    return shared_from_this();
}

std::shared_ptr<Term> Variable::derivative(shared_ptr<Variable> v) {
    if (shared_from_this() == v) {
        return TermBuilder::constant(1);
    } else {
        return TermBuilder::constant(0);
    }
}

std::string Variable::toString() {
    std::string str;
    if (ownId < 0) {
        str.append("Var_");
        str.append(std::to_string(-ownId));
    } else {
        str.append("Var");
        str.append(std::to_string(ownId));
    }
    return str;
}
} /* namespace autodiff */
