/*
 * CompiledAnd.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledAnd.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff {
namespace compiled {
void CompiledAnd::accept(shared_ptr<ITapeVisitor> visitor) {
    shared_ptr<CompiledAnd> thisCasted = dynamic_pointer_cast<CompiledAnd>(shared_from_this());
    visitor->visit(thisCasted);
}
} /* namespace compiled */
} /* namespace autodiff */
