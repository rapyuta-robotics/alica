/*
 * CompiledSum.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#include "compiled/CompiledSum.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledSum::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledSum> thisCasted = dynamic_pointer_cast<CompiledSum>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
