/*
 * Zero.h
 *
 *  Created on: Jun 6, 2014
 *      Author: psp
 */

#ifndef ZERO_H_
#define ZERO_H_

#include "Term.h"

#include <iostream>

using namespace std;

namespace autodiff
{
	class Zero : public Term
	{
	public:
		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		string toString();
	};

} /* namespace autodiff */

#endif /* ZERO_H_ */
