/*
 * TermPower.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef TERMPOWER_H_
#define TERMPOWER_H_

#include "Term.h"

namespace AutoDiff
{

	class TermPower : public Term
	{
	public:
		TermPower(shared_ptr<Term> baseTerm, shared_ptr<Term> exponent);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		const shared_ptr<Term> getBase();
		const shared_ptr<Term> getExponent();
	private:
		shared_ptr<Term> _base;
		shared_ptr<Term> _exponent;
	};

} /* namespace AutoDiff */

#endif /* TERMPOWER_H_ */
