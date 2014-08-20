/*
 * CompiledLTEConstraint.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDLTECONSTRAINT_H_
#define COMPILEDLTECONSTRAINT_H_

#include "TapeElement.h"

namespace AutoDiff
{
	namespace Compiled
	{

		class CompiledLTEConstraint : public TapeElement
		{
		public:
			int _left;
			int _right;
			double _steepness;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDLTECONSTRAINT_H_ */
