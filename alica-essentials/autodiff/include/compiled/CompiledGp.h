/*
 * CompiledGp.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDGP_H_
#define COMPILEDGP_H_

#include "TapeElement.h"

namespace AutoDiff
{
	namespace Compiled
	{

		class CompiledGp : public TapeElement
		{
			// TODO:

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDGP_H_ */
