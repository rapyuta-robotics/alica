/*
 * CompiledVariable.h
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#ifndef COMPILEDVARIABLE_H_
#define COMPILEDVARIABLE_H_

#include "TapeElement.h"

#include <memory>

using namespace std;

namespace AutoDiff
{
	namespace Compiled
	{
		class CompiledVariable : public TapeElement
		{
		public:
			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDVARIABLE_H_ */
