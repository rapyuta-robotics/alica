/*
 * CompiledExp.h
 *
 *  Created on: Jul 15, 2014
 *      Author: psp
 */

#ifndef COMPILEDEXP_H_
#define COMPILEDEXP_H_

#include "TapeElement.h"

namespace autodiff
{
	namespace compiled
	{

		class CompiledExp : public TapeElement
		{
		public:
			int _arg;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDEXP_H_ */
