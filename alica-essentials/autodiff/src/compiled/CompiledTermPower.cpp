/*
 * CompiledTermPower.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledTermPower.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		void CompiledTermPower::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledTermPower> thisCasted = dynamic_pointer_cast<CompiledTermPower>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
