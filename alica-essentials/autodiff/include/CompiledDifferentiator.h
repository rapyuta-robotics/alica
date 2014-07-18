/*
 * CompiledDifferentiator.h
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#ifndef COMPILEDDIFFERENTIATOR_H_
#define COMPILEDDIFFERENTIATOR_H_

#include "ICompiledTerm.h"
#include "ITermVisitor.h"
#include "compiled/ITapeVisitor.h"

#include <map>
#include <memory>
#include <functional>

using namespace std;

namespace AutoDiff
{
	class Term;

	namespace Compiled
	{
		class TapeElement;
	}

	class CompiledDifferentiator : public ICompiledTerm
	{
	public:
		CompiledDifferentiator(shared_ptr<Term> function, vector<shared_ptr<Variable>> variables);

		virtual double evaluate(vector<double> arg);
		virtual pair<vector<double>, double> differentiate(vector<double> arg);

	private:
		vector<shared_ptr<Compiled::TapeElement>> _tape;
		int _dimension;
		vector<shared_ptr<Variable>> _variables;

		void forwardSweep(vector<double> arg);
		void reverseSweep();
		void evaluateTape(vector<double> arg);

		class Compiler : public ITermVisitor
		{
		public:
			Compiler(vector<shared_ptr<Variable>> variables, vector<shared_ptr<Compiled::TapeElement>>* tape);
			~Compiler();

			void compile(shared_ptr<Term> term);

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
		private:
			vector<shared_ptr<Compiled::TapeElement>> *_tape;
			map<int, int> _indexOf;

			int compile(shared_ptr<Term> term, function<shared_ptr<Compiled::TapeElement> ()> compiler);
		};

		class EvalVisitor : public Compiled::ITapeVisitor
		{
		public:
			EvalVisitor(vector<shared_ptr<Compiled::TapeElement>> *tape);

			void visit(shared_ptr<Compiled::CompiledAbs> elem);
			void visit(shared_ptr<Compiled::CompiledAnd> elem);
			void visit(shared_ptr<Compiled::CompiledAtan2> elem);
			void visit(shared_ptr<Compiled::CompiledConstant> elem);
			void visit(shared_ptr<Compiled::CompiledConstPower> elem);
			void visit(shared_ptr<Compiled::CompiledConstraintUtility> elem);
			void visit(shared_ptr<Compiled::CompiledCos> elem);
			void visit(shared_ptr<Compiled::CompiledExp> elem);
			void visit(shared_ptr<Compiled::CompiledGp> elem);
			void visit(shared_ptr<Compiled::CompiledLinSigmoid> elem);
			void visit(shared_ptr<Compiled::CompiledLog> elem);
			void visit(shared_ptr<Compiled::CompiledLTConstraint> elem);
			void visit(shared_ptr<Compiled::CompiledLTEConstraint> elem);
			void visit(shared_ptr<Compiled::CompiledMax> elem);
			void visit(shared_ptr<Compiled::CompiledMin> elem);
			void visit(shared_ptr<Compiled::CompiledOr> elem);
			void visit(shared_ptr<Compiled::CompiledProduct> elem);
			void visit(shared_ptr<Compiled::CompiledReification> elem);
			void visit(shared_ptr<Compiled::CompiledSigmoid> elem);
			void visit(shared_ptr<Compiled::CompiledSin> elem);
			void visit(shared_ptr<Compiled::CompiledSum> elem);
			void visit(shared_ptr<Compiled::CompiledTermPower> elem);
			void visit(shared_ptr<Compiled::CompiledVariable> var);
		private:
			vector<shared_ptr<Compiled::TapeElement>> *_tape;

			double valueOf(int index);
		};

		class ForwardSweepVisitor : public Compiled::ITapeVisitor
		{
		public:
			ForwardSweepVisitor(vector<shared_ptr<Compiled::TapeElement>> *tape);

			void visit(shared_ptr<Compiled::CompiledAbs> elem);
			void visit(shared_ptr<Compiled::CompiledAnd> elem);
			void visit(shared_ptr<Compiled::CompiledAtan2> elem);
			void visit(shared_ptr<Compiled::CompiledConstant> elem);
			void visit(shared_ptr<Compiled::CompiledConstPower> elem);
			void visit(shared_ptr<Compiled::CompiledConstraintUtility> elem);
			void visit(shared_ptr<Compiled::CompiledCos> elem);
			void visit(shared_ptr<Compiled::CompiledExp> elem);
			void visit(shared_ptr<Compiled::CompiledGp> elem);
			void visit(shared_ptr<Compiled::CompiledLinSigmoid> elem);
			void visit(shared_ptr<Compiled::CompiledLog> elem);
			void visit(shared_ptr<Compiled::CompiledLTConstraint> elem);
			void visit(shared_ptr<Compiled::CompiledLTEConstraint> elem);
			void visit(shared_ptr<Compiled::CompiledMax> elem);
			void visit(shared_ptr<Compiled::CompiledMin> elem);
			void visit(shared_ptr<Compiled::CompiledOr> elem);
			void visit(shared_ptr<Compiled::CompiledProduct> elem);
			void visit(shared_ptr<Compiled::CompiledReification> elem);
			void visit(shared_ptr<Compiled::CompiledSigmoid> elem);
			void visit(shared_ptr<Compiled::CompiledSin> elem);
			void visit(shared_ptr<Compiled::CompiledSum> elem);
			void visit(shared_ptr<Compiled::CompiledTermPower> elem);
			void visit(shared_ptr<Compiled::CompiledVariable> var);
		private:
			vector<shared_ptr<Compiled::TapeElement>> *_tape;

			double valueOf(int index);
		};
	};

} /* namespace AutoDiff */

#endif /* COMPILEDDIFFERENTIATOR_H_ */
