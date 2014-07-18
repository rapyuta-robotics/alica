/*
 * CompiledGp.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledGp.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		void CompiledGp::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledGp> thisCasted = dynamic_pointer_cast<CompiledGp>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
