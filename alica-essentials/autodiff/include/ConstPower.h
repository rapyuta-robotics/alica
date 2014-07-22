/*
 * ConstPower.h
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#ifndef CONSTPOWER_H_
#define CONSTPOWER_H_

#include "Term.h"

namespace AutoDiff
{

	class ConstPower : public Term
	{
	public:
		ConstPower(shared_ptr<Term> baseTerm, double exponent);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		const shared_ptr<Term> getBase();
		const double getExponent();
	private:
		shared_ptr<Term> _base;
		double _exponent;
	};

} /* namespace AutoDiff */

#endif /* CONSTPOWER_H_ */
