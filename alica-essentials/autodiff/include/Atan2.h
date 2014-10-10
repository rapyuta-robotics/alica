/*
 * Atan2.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef ATAN2_H_
#define ATAN2_H_

#include "Term.h"

namespace autodiff
{

	class Atan2 : public Term
	{
	public:
		Atan2(shared_ptr<Term> left, shared_ptr<Term> right);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		const shared_ptr<Term> getLeft();
		const shared_ptr<Term> getRight();

	private:
		shared_ptr<Term> _left;
		shared_ptr<Term> _right;
	};

} /* namespace autodiff */

#endif /* ATAN2_H_ */
