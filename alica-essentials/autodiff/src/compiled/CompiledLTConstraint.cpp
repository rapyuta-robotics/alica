/*
 * CompiledLTConstraint.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledLTConstraint.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff {
namespace compiled {
void CompiledLTConstraint::accept(shared_ptr<ITapeVisitor> visitor) {
    shared_ptr<CompiledLTConstraint> thisCasted = dynamic_pointer_cast<CompiledLTConstraint>(shared_from_this());
    visitor->visit(thisCasted);
}
} /* namespace compiled */
} /* namespace autodiff */
