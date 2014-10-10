/*
 * CompiledMax.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDMAX_H_
#define COMPILEDMAX_H_

#include "TapeElement.h"

namespace autodiff
{
	namespace compiled
	{

		class CompiledMax : public TapeElement
		{
		public:
			int _left;
			int _right;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDMAX_H_ */
