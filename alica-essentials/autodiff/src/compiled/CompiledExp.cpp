/*
 * CompiledExp.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: psp
 */

#include "compiled/CompiledExp.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		void CompiledExp::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledExp> thisCasted = dynamic_pointer_cast<CompiledExp>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
