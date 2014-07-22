/*
 * CompiledExp.h
 *
 *  Created on: Jul 15, 2014
 *      Author: psp
 */

#ifndef COMPILEDEXP_H_
#define COMPILEDEXP_H_

#include "TapeElement.h"

namespace AutoDiff
{
	namespace Compiled
	{

		class CompiledExp : public TapeElement
		{
		public:
			int _arg;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDEXP_H_ */
