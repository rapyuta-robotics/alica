/*
 * CompiledAtan2.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDATAN2_H_
#define COMPILEDATAN2_H_

#include "TapeElement.h"

namespace AutoDiff
{
	namespace Compiled
	{

		class CompiledAtan2 : public TapeElement
		{
		public:
			int _left;
			int _right;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDATAN2_H_ */
