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

namespace autodiff
{
	namespace compiled
	{
		class CompiledProduct : public TapeElement
		{
		public:
			int _left;
			int _right;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDPRODUCT_H_ */
