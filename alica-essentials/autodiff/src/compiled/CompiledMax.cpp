/*
 * CompiledMax.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledMax.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledMax::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledMax> thisCasted = dynamic_pointer_cast<CompiledMax>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
