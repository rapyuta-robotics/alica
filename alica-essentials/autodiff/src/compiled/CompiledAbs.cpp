/*
 * CompiledAbs.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledAbs.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledAbs::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledAbs> thisCasted = dynamic_pointer_cast<CompiledAbs>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
