/*
 * CompiledVariable.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#include "compiled/CompiledVariable.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledVariable::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledVariable> thisCasted = dynamic_pointer_cast<CompiledVariable>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
