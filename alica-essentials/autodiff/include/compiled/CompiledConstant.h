/*
 * CompiledConstant.h
 *
 *  Created on: Jul 15, 2014
 *      Author: psp
 */

#ifndef COMPILEDCONSTANT_H_
#define COMPILEDCONSTANT_H_

#include "TapeElement.h"

namespace autodiff
{
	namespace compiled
	{

		class CompiledConstant : public TapeElement
		{
		public:
			CompiledConstant(double value);

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDCONSTANT_H_ */
