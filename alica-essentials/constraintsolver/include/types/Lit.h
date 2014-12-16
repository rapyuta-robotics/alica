/*
 * Lit.h
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#ifndef LIT_H_
#define LIT_H_

#include <AutoDiff.h>

#include "types/Assignment.h"

#include <memory>

using namespace std;
using namespace autodiff;

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{
			class Var;

			class Lit : public ITermVisitor
			{
			public:
				Lit(shared_ptr<Var> v, Assignment ass = Assignment::TRUE);
				Lit(shared_ptr<Term> t, Assignment ass, bool temp);
				virtual ~Lit();

				bool satisfied();
				bool conflicted();
				void computeVariableCount();

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

				Assignment sign;
				shared_ptr<Var> var;
				int variableCount;
				bool isTemporary;
				shared_ptr<Term> atom;
			};

		} /* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */

#endif /* LIT_H_ */
