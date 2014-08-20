/*
 * CompiledSigmoid.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledSigmoid.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		void CompiledSigmoid::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledSigmoid> thisCasted = dynamic_pointer_cast<CompiledSigmoid>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
