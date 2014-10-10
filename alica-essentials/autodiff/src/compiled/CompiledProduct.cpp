/*
 * CompiledProduct.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: psp
 */

#include "compiled/CompiledProduct.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledProduct::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledProduct> thisCasted = dynamic_pointer_cast<CompiledProduct>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
