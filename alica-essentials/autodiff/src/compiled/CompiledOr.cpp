/*
 * CompiledOr.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledOr.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledOr::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledOr> thisCasted = dynamic_pointer_cast<CompiledOr>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
