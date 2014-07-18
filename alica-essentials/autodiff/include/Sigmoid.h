/*
 * Sigmoid.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef SIGMOID_H_
#define SIGMOID_H_

#include "Term.h"

namespace AutoDiff
{

	class Sigmoid : public Term
	{
	public:
		Sigmoid(shared_ptr<Term> arg, shared_ptr<Term> mid);
		Sigmoid(shared_ptr<Term> arg, shared_ptr<Term> mid, double steppness);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		const shared_ptr<Term> getArg();
		const shared_ptr<Term> getMid();
		const double getSteepness();
	private:
		shared_ptr<Term> _arg;
		shared_ptr<Term> _mid;
		double _steepness;
	};

} /* namespace AutoDiff */

#endif /* SIGMOID_H_ */
