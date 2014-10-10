/*
 * CompiledAbs.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDABS_H_
#define COMPILEDABS_H_

#include "TapeElement.h"

namespace autodiff
{
	namespace compiled
	{

		class CompiledAbs : public TapeElement
		{
		public:
			int _arg;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDABS_H_ */
