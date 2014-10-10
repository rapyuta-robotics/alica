/*
 * CompiledGp.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDGP_H_
#define COMPILEDGP_H_

#include "TapeElement.h"

#include <memory>
#include <vector>

namespace autodiff
{
	class GaussianProcess; // XXX: class is missing, TODO: import

	namespace compiled
	{

		class CompiledGp : public TapeElement
		{
		public:
			vector<int> _terms;
			int _dc;
			shared_ptr<GaussianProcess> _gpr;

			void accept(shared_ptr<ITapeVisitor> visitor);
		};

	} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDGP_H_ */
