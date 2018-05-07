/*
 * CompiledTermPower.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledTermPower.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff {
namespace compiled {
void CompiledTermPower::accept(shared_ptr<ITapeVisitor> visitor) {
    shared_ptr<CompiledTermPower> thisCasted = dynamic_pointer_cast<CompiledTermPower>(shared_from_this());
    visitor->visit(thisCasted);
}
} /* namespace compiled */
} /* namespace autodiff */
