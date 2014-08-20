/*
 * CompiledReification.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledReification.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		void CompiledReification::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledReification> thisCasted = dynamic_pointer_cast<CompiledReification>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
