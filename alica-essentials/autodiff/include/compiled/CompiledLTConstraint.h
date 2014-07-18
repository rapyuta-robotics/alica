/*
 * CompiledLTConstraint.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDLTCONSTRAINT_H_
#define COMPILEDLTCONSTRAINT_H_

#include "TapeElement.h"

namespace AutoDiff
{
	namespace Compiled
	{

		class CompiledLTConstraint : public TapeElement
		{
		public:
			int _left;
			int _right;
			double _steepness;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDLTCONSTRAINT_H_ */
