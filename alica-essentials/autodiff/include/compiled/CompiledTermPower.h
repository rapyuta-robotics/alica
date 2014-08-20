/*
 * CompiledTermPower.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDTERMPOWER_H_
#define COMPILEDTERMPOWER_H_

#include "TapeElement.h"

namespace AutoDiff
{
	namespace Compiled
	{

		class CompiledTermPower : public TapeElement
		{
		public:
			int _base;
			int _exponent;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDTERMPOWER_H_ */
