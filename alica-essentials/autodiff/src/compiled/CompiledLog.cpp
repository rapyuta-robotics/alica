/*
 * CompiledLog.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "compiled/CompiledLog.h"

#include "compiled/ITapeVisitor.h"

namespace autodiff
{
	namespace compiled
	{
		void CompiledLog::accept(shared_ptr<ITapeVisitor> visitor)
		{
			shared_ptr<CompiledLog> thisCasted = dynamic_pointer_cast<CompiledLog>(shared_from_this());
			visitor->visit(thisCasted);
		}
	} /* namespace compiled */
} /* namespace autodiff */
