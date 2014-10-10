/*
 * CompiledLTEConstraint.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledLTEConstraint.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledLTEConstraint::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledLTEConstraint> thisCasted = dynamic_pointer_cast<CompiledLTEConstraint>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
