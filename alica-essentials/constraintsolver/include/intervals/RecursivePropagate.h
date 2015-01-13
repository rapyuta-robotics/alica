/*
 * RecursivePropagate.h
 *
 *  Created on: Dec 11, 2014
 *      Author: Philipp
 */

#ifndef RECURSIVEPROPAGATE_H_
#define RECURSIVEPROPAGATE_H_

#include <AutoDiff.h>

#include <memory>

using namespace std;
using namespace autodiff;

namespace alica
{
	namespace reasoner
	{
		namespace intervalpropagation
		{
			class DownwardPropagator;
			class SetParents;
			class TermList;
			class UpwardPropagator;

			class RecursivePropagate : public ITermVisitor
			{
			public:
				RecursivePropagate();
				virtual ~RecursivePropagate();

				bool propagate(shared_ptr<Term> term);
				void addToQueue(shared_ptr<Term> t);

				int visit(shared_ptr<Abs> abs);
				int visit(shared_ptr<And> and_);
				int visit(shared_ptr<Atan2> atan2);
				int visit(shared_ptr<Constant> constant);
				int visit(shared_ptr<ConstPower> intPower);
				int visit(shared_ptr<ConstraintUtility> cu);
				int visit(shared_ptr<Cos> cos);
				int visit(shared_ptr<Exp> exp);
				int visit(shared_ptr<Gp> gp);
				int visit(shared_ptr<LinSigmoid> sigmoid);
				int visit(shared_ptr<Log> log);
				int visit(shared_ptr<LTConstraint> constraint);
				int visit(shared_ptr<LTEConstraint> constraint);
				int visit(shared_ptr<Max> max);
				int visit(shared_ptr<Min> min);
				int visit(shared_ptr<Or> or_);
				int visit(shared_ptr<Product> product);
				int visit(shared_ptr<Reification> dis);
				int visit(shared_ptr<Sigmoid> sigmoid);
				int visit(shared_ptr<Sin> sin);
				int visit(shared_ptr<Sum> sum);
				int visit(shared_ptr<TermPower> power);
				int visit(shared_ptr<Variable> var);
				int visit(shared_ptr<Zero> zero);

			protected:
				shared_ptr<TermList> changed;

				shared_ptr<DownwardPropagator> dp;
				shared_ptr<UpwardPropagator> up;
				shared_ptr<SetParents> sp;
			};

		} /* namespace intervalpropagation */
	} /* namespace reasoner */
} /* namespace alica */

#endif /* RECURSIVEPROPAGATE_H_ */
