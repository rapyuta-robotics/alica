/*
 * CompiledMax.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledMax.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		void CompiledMax::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledMax> thisCasted = dynamic_pointer_cast<CompiledMax>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
