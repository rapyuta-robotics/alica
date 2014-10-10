/*
 * CompiledSigmoid.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDSIGMOID_H_
#define COMPILEDSIGMOID_H_

#include "TapeElement.h"

namespace autodiff
{
	namespace compiled
	{

		class CompiledSigmoid : public TapeElement
		{
		public:
			int _arg;
			int _mid;
			double _steepness;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDSIGMOID_H_ */
