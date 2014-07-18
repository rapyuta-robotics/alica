/*
 * CompiledProduct.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: psp
 */

#include "compiled/CompiledProduct.h"

#include "compiled/ITapeVisitor.h"

namespace AutoDiff
{
	namespace Compiled
	{
		void CompiledProduct::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledProduct> thisCasted = dynamic_pointer_cast<CompiledProduct>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace Compiled */
} /* namespace AutoDiff */
