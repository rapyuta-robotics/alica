/*
 * CompiledAtan2.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledAtan2.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledAtan2::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledAtan2> thisCasted = dynamic_pointer_cast<CompiledAtan2>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
