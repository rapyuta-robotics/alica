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

namespace AutoDiff
{
	class GaussianProcess; // XXX: class is missing, TODO: import

	class Gp : public Term
	{
	public:
		Gp(vector<shared_ptr<Term>> args, shared_ptr<GaussianProcess> gp, int dc);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		double eval();

		const vector<shared_ptr<Term>> getArgs();
		const shared_ptr<GaussianProcess> getGpr();
		const int getDivCount();
	private:
		vector<shared_ptr<Term>> _args;
		shared_ptr<GaussianProcess> _gpr;
		int _divCount;
	};

} /* namespace AutoDiff */

#endif /* GP_H_ */
