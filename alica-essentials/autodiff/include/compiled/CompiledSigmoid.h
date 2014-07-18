/*
 * CompiledSigmoid.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDSIGMOID_H_
#define COMPILEDSIGMOID_H_

#include "TapeElement.h"

namespace AutoDiff
{
	namespace Compiled
	{

		class CompiledSigmoid : public TapeElement
		{
		public:
			int _arg;
			int _mid;
			double _steepness;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDSIGMOID_H_ */
