/*
 * ITapevisitor.h
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#ifndef ITAPEVISITOR_H_
#define ITAPEVISITOR_H_

#include <memory>

using namespace std;

namespace autodiff
{
	namespace compiled
	{
		class CompiledAbs;
		class CompiledAnd;
		class CompiledAtan2;
		class CompiledConstant;
		class CompiledConstPower;
		class CompiledConstraintUtility;
		class CompiledCos;
		class CompiledExp;
		class CompiledGp;
		class CompiledLinSigmoid;
		class CompiledLog;
		class CompiledLTConstraint;
		class CompiledLTEConstraint;
		class CompiledMax;
		class CompiledMin;
		class CompiledOr;
		class CompiledProduct;
		class CompiledReification;
		class CompiledSigmoid;
		class CompiledSin;
		class CompiledSum;
		class CompiledTermPower;
		class CompiledVariable;

		class ITapeVisitor
		{
		public:
			virtual ~ITapeVisitor()
			{
			}

			virtual void visit(shared_ptr<CompiledAbs> elem) = 0;
			virtual void visit(shared_ptr<CompiledAnd> elem) = 0;
			virtual void visit(shared_ptr<CompiledAtan2> elem) = 0;
			virtual void visit(shared_ptr<CompiledConstant> elem) = 0;
			virtual void visit(shared_ptr<CompiledConstPower> elem) = 0;
			virtual void visit(shared_ptr<CompiledConstraintUtility> elem) = 0;
			virtual void visit(shared_ptr<CompiledCos> elem) = 0;
			virtual void visit(shared_ptr<CompiledExp> elem) = 0;
			virtual void visit(shared_ptr<CompiledGp> elem) = 0;
			virtual void visit(shared_ptr<CompiledLinSigmoid> elem) = 0;
			virtual void visit(shared_ptr<CompiledLog> elem) = 0;
			virtual void visit(shared_ptr<CompiledLTConstraint> elem) = 0;
			virtual void visit(shared_ptr<CompiledLTEConstraint> elem) = 0;
			virtual void visit(shared_ptr<CompiledMax> elem) = 0;
			virtual void visit(shared_ptr<CompiledMin> elem) = 0;
			virtual void visit(shared_ptr<CompiledOr> elem) = 0;
			virtual void visit(shared_ptr<CompiledProduct> elem) = 0;
			virtual void visit(shared_ptr<CompiledReification> elem) = 0;
			virtual void visit(shared_ptr<CompiledSigmoid> elem) = 0;
			virtual void visit(shared_ptr<CompiledSin> elem) = 0;
			virtual void visit(shared_ptr<CompiledSum> elem) = 0;
			virtual void visit(shared_ptr<CompiledTermPower> elem) = 0;
			virtual void visit(shared_ptr<CompiledVariable> var) = 0;
		};
	} // namespace compiled

} /* namespace autodiff */

#endif /* ITAPEVISITOR_H_ */
