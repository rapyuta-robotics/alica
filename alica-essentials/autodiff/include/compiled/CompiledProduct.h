/*
 * CompiledProduct.h
 *
 *  Created on: Jul 15, 2014
 *      Author: psp
 */

#ifndef COMPILEDPRODUCT_H_
#define COMPILEDPRODUCT_H_

#include "TapeElement.h"

#include <memory>

using namespace std;

namespace AutoDiff
{
	namespace Compiled
	{
		class CompiledProduct : public TapeElement
		{
		public:
			int _left;
			int _right;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDPRODUCT_H_ */
