/*
 * LinSigmoid.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef LINSIGMOID_H_
#define LINSIGMOID_H_

#include "Term.h"

namespace AutoDiff
{

	class LinSigmoid : public Term
	{
	public:
		LinSigmoid(shared_ptr<Term> arg);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		const shared_ptr<Term> getArg();

	private:
		shared_ptr<Term> _arg;
	};

} /* namespace AutoDiff */

#endif /* LINSIGMOID_H_ */
