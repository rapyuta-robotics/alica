/*
 * CompiledCos.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledCos.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledCos::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledCos> thisCasted = dynamic_pointer_cast<CompiledCos>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
