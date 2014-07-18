/*
 * Variable.h
 *
 *  Created on: Jun 5, 2014
 *      Author: psp
 */

#ifndef AutoDiffVARIABLE_H_
#define AutoDiffVARIABLE_H_

#include "Term.h"

namespace AutoDiff
{
	class Variable : public Term
	{
	public:
		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);
	};
} /* namespace AutoDiff */

#endif /* AutoDiffVARIABLE_H_ */
