/*
 * CompiledConstraintUtility.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledConstraintUtility.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledConstraintUtility::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledConstraintUtility> thisCasted = dynamic_pointer_cast<CompiledConstraintUtility>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
