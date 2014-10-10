/*
 * ITermVisitor.h
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#ifndef ITERMVISITOR_H_
#define ITERMVISITOR_H_

#include <memory>

using namespace std;

namespace autodiff
{
	class Abs;
	class And;
	class Atan2;
	class Constant;
	class ConstPower;
	class ConstraintUtility;
	class Cos;
	class Exp;
	class Gp;
	class LinSigmoid;
	class Log;
	class LTConstraint;
	class LTEConstraint;
	class Max;
	class Min;
	class Or;
	class Product;
	class Reification;
	class Sigmoid;
	class Sin;
	class Sum;
	class TermPower;
	class Variable;
	class Zero;

	class ITermVisitor : public enable_shared_from_this<ITermVisitor>
	{
	public:
		virtual ~ITermVisitor()
		{
		}

		virtual int visit(shared_ptr<Abs> elem) = 0;
		virtual int visit(shared_ptr<And> elem) = 0;
		virtual int visit(shared_ptr<Atan2> elem) = 0;
		virtual int visit(shared_ptr<Constant> elem) = 0;
		virtual int visit(shared_ptr<ConstPower> elem) = 0;
		virtual int visit(shared_ptr<ConstraintUtility> elem) = 0;
		virtual int visit(shared_ptr<Cos> elem) = 0;
		virtual int visit(shared_ptr<Exp> elem) = 0;
		virtual int visit(shared_ptr<Gp> elem) = 0;
		virtual int visit(shared_ptr<LinSigmoid> elem) = 0;
		virtual int visit(shared_ptr<Log> elem) = 0;
		virtual int visit(shared_ptr<LTConstraint> elem) = 0;
		virtual int visit(shared_ptr<LTEConstraint> elem) = 0;
		virtual int visit(shared_ptr<Max> elem) = 0;
		virtual int visit(shared_ptr<Min> elem) = 0;
		virtual int visit(shared_ptr<Or> elem) = 0;
		virtual int visit(shared_ptr<Product> elem) = 0;
		virtual int visit(shared_ptr<Reification> elem) = 0;
		virtual int visit(shared_ptr<Sigmoid> elem) = 0;
		virtual int visit(shared_ptr<Sin> elem) = 0;
		virtual int visit(shared_ptr<Sum> elem) = 0;
		virtual int visit(shared_ptr<TermPower> elem) = 0;
		virtual int visit(shared_ptr<Variable> var) = 0;
		virtual int visit(shared_ptr<Zero> elem) = 0;
	};

} /* namespace autodiff */

#endif /* ITERMVISITOR_H_ */
