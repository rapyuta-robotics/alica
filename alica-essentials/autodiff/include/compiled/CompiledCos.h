/*
 * CompiledCos.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDCOS_H_
#define COMPILEDCOS_H_

#include "TapeElement.h"

namespace autodiff
{
	namespace compiled
	{

		class CompiledCos : public TapeElement
		{
		public:
			int _arg;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDCOS_H_ */
