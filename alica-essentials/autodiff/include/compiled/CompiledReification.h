/*
 * CompiledReification.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDREIFICATION_H_
#define COMPILEDREIFICATION_H_

#include "TapeElement.h"

namespace AutoDiff
{
	namespace Compiled
	{

		class CompiledReification : public TapeElement
		{
		public:
			double _min;
			double _max;
			int _condition;
			int _negatedCondition;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDREIFICATION_H_ */
