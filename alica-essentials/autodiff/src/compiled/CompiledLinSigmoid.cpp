/*
 * CompiledLinSigmoid.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledLinSigmoid.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff {
namespace compiled {
void CompiledLinSigmoid::accept(shared_ptr<ITapeVisitor> visitor) {
    shared_ptr<CompiledLinSigmoid> thisCasted = dynamic_pointer_cast<CompiledLinSigmoid>(shared_from_this());
    visitor->visit(thisCasted);
}
} /* namespace compiled */
} /* namespace autodiff */
