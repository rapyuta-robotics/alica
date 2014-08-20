/*
 * CompiledConstPower.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledConstPower.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		void CompiledConstPower::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledConstPower> thisCasted = dynamic_pointer_cast<CompiledConstPower>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
