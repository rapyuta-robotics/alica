/*
 * CompiledMin.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDMIN_H_
#define COMPILEDMIN_H_

#include "TapeElement.h"

namespace autodiff
{
	namespace compiled
	{

		class CompiledMin : public TapeElement
		{
		public:
			int _left;
			int _right;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDMIN_H_ */
