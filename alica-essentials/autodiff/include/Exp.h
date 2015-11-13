/*
 * Exp.h
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#ifndef EXP_H_
#define EXP_H_

#include "Term.h"

namespace autodiff
{

	class Exp : public Term
	{
	public:
		Exp(shared_ptr<Term> arg);

		shared_ptr<Term> arg;

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		string toString();
	};

} /* namespace autodiff */

#endif /* EXP_H_ */
