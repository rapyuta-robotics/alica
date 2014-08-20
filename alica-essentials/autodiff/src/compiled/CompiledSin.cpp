/*
 * CompiledSin.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledSin.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		void CompiledSin::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledSin> thisCasted = dynamic_pointer_cast<CompiledSin>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
