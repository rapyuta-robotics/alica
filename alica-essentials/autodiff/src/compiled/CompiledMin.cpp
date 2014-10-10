/*
 * CompiledMin.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledMin.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledMin::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledMin> thisCasted = dynamic_pointer_cast<CompiledMin>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
