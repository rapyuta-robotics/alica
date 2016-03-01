/*
 * ConstPower.h
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#ifndef CONSTPOWER_H_
#define CONSTPOWER_H_

#include "Term.h"

#include <iostream>

using namespace std;

namespace autodiff
{

	class ConstPower : public Term
	{
	public:
		ConstPower(shared_ptr<Term> baseTerm, double exponent);

		shared_ptr<Term> base;
		double exponent;

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		string toString();
	};

} /* namespace autodiff */

#endif /* CONSTPOWER_H_ */
