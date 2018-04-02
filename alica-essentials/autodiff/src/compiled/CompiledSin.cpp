/*
 * CompiledSin.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledSin.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff {
namespace compiled {
void CompiledSin::accept(shared_ptr<ITapeVisitor> visitor) {
    shared_ptr<CompiledSin> thisCasted = dynamic_pointer_cast<CompiledSin>(shared_from_this());
    visitor->visit(thisCasted);
}
} /* namespace compiled */
} /* namespace autodiff */
