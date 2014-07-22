/*
 * CompiledSum.h
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#ifndef COMPILEDSUM_H_
#define COMPILEDSUM_H_

#include "TapeElement.h"

#include <memory>
#include <vector>

using namespace std;

namespace AutoDiff
{
	namespace Compiled
	{
		class CompiledSum : public TapeElement
		{
		public:
			vector<int> _terms;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};
	} /* namespace Compiled */
} /* namespace AutoDiff */

#endif /* COMPILEDSUM_H_ */
