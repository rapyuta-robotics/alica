/*
 * CompiledConstant.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: psp
 */

#include "compiled/CompiledConstant.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		CompiledConstant::CompiledConstant(double value) {
			this->value = value;
		}

		void CompiledConstant::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledConstant> thisCasted = dynamic_pointer_cast<CompiledConstant>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
