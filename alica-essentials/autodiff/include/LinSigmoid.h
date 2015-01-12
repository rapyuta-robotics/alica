/*
 * LinSigmoid.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef LINSIGMOID_H_
#define LINSIGMOID_H_

#include "Term.h"

namespace autodiff
{

	class LinSigmoid : public Term
	{
	public:
		LinSigmoid(shared_ptr<Term> arg);

		shared_ptr<Term> arg;

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);
	};

} /* namespace autodiff */

#endif /* LINSIGMOID_H_ */
