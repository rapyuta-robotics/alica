/*
 * CompiledSigmoid.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledSigmoid.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff {
namespace compiled {
void CompiledSigmoid::accept(shared_ptr<ITapeVisitor> visitor) {
    shared_ptr<CompiledSigmoid> thisCasted = dynamic_pointer_cast<CompiledSigmoid>(shared_from_this());
    visitor->visit(thisCasted);
}
} /* namespace compiled */
} /* namespace autodiff */
