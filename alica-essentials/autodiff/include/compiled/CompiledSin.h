/*
 * CompiledSin.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDSIN_H_
#define COMPILEDSIN_H_

#include "TapeElement.h"

namespace autodiff
{
	namespace compiled
	{

		class CompiledSin : public TapeElement
		{
		public:
			int _arg;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDSIN_H_ */
