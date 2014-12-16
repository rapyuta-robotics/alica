/*
 * Gp.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef GP_H_
#define GP_H_

#include "Term.h"

#include <vector>

namespace autodiff
{
	class GaussianProcess; // XXX: class is missing, TODO: import

	class Gp : public Term
	{
	public:
		Gp(vector<shared_ptr<Term>> args, shared_ptr<GaussianProcess> gp, int dc);

		vector<shared_ptr<Term>> args;
		shared_ptr<GaussianProcess> gpr;
		int divCount;

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		double eval();
	};

} /* namespace autodiff */

#endif /* GP_H_ */
