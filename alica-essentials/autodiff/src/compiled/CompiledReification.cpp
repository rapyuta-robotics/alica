/*
 * CompiledReification.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledReification.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledReification::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledReification> thisCasted = dynamic_pointer_cast<CompiledReification>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
