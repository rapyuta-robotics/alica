/*
 * Constant.h
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#ifndef CONSTANT_H_
#define CONSTANT_H_

#include "Term.h"

namespace autodiff
{

	class Constant : public Term
	{
	public:
		Constant(double value);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		double getValue();
	private:
		double _value;
	};

} /* namespace autodiff */

#endif /* CONSTANT_H_ */
