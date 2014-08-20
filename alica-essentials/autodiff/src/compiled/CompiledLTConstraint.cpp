/*
 * CompiledLTConstraint.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledLTConstraint.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		void CompiledLTConstraint::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledLTConstraint> thisCasted = dynamic_pointer_cast<CompiledLTConstraint>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
