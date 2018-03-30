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

namespace autodiff {
using namespace compiled;

class Term;

namespace compiled {
class TapeElement;
}

class CompiledDifferentiator : public ICompiledTerm {
public:
    CompiledDifferentiator(shared_ptr<Term> function, shared_ptr<vector<shared_ptr<Variable>>> variables);

    virtual double evaluate(shared_ptr<vector<double>> arg);
    virtual pair<shared_ptr<vector<double>>, double> differentiate(shared_ptr<vector<double>> arg);

private:
    vector<shared_ptr<TapeElement>> _tape;
    int _dimension;
    shared_ptr<vector<shared_ptr<Variable>>> _variables;

    void forwardSweep(shared_ptr<vector<double>> arg);
    void reverseSweep();
    void evaluateTape(shared_ptr<vector<double>> arg);

    class Compiler : public ITermVisitor {
    public:
        Compiler(shared_ptr<vector<shared_ptr<Variable>>> variables, vector<shared_ptr<TapeElement>>* tape);
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
        vector<shared_ptr<TapeElement>>* _tape;
        map<int, int> _indexOf;

        int compile(shared_ptr<Term> term, function<shared_ptr<TapeElement>()> compiler);
    };

    class EvalVisitor : public ITapeVisitor {
    public:
        EvalVisitor(vector<shared_ptr<TapeElement>>* tape);

        void visit(shared_ptr<CompiledAbs> elem);
        void visit(shared_ptr<CompiledAnd> elem);
        void visit(shared_ptr<CompiledAtan2> elem);
        void visit(shared_ptr<CompiledConstant> elem);
        void visit(shared_ptr<CompiledConstPower> elem);
        void visit(shared_ptr<CompiledConstraintUtility> elem);
        void visit(shared_ptr<CompiledCos> elem);
        void visit(shared_ptr<CompiledExp> elem);
        void visit(shared_ptr<CompiledGp> elem);
        void visit(shared_ptr<CompiledLinSigmoid> elem);
        void visit(shared_ptr<CompiledLog> elem);
        void visit(shared_ptr<CompiledLTConstraint> elem);
        void visit(shared_ptr<CompiledLTEConstraint> elem);
        void visit(shared_ptr<CompiledMax> elem);
        void visit(shared_ptr<CompiledMin> elem);
        void visit(shared_ptr<CompiledOr> elem);
        void visit(shared_ptr<CompiledProduct> elem);
        void visit(shared_ptr<CompiledReification> elem);
        void visit(shared_ptr<CompiledSigmoid> elem);
        void visit(shared_ptr<CompiledSin> elem);
        void visit(shared_ptr<CompiledSum> elem);
        void visit(shared_ptr<CompiledTermPower> elem);
        void visit(shared_ptr<CompiledVariable> var);

    private:
        vector<shared_ptr<TapeElement>>* _tape;

        double valueOf(int index);
    };

    class ForwardSweepVisitor : public ITapeVisitor {
    public:
        ForwardSweepVisitor(vector<shared_ptr<TapeElement>>* tape);

        void visit(shared_ptr<CompiledAbs> elem);
        void visit(shared_ptr<CompiledAnd> elem);
        void visit(shared_ptr<CompiledAtan2> elem);
        void visit(shared_ptr<CompiledConstant> elem);
        void visit(shared_ptr<CompiledConstPower> elem);
        void visit(shared_ptr<CompiledConstraintUtility> elem);
        void visit(shared_ptr<CompiledCos> elem);
        void visit(shared_ptr<CompiledExp> elem);
        void visit(shared_ptr<CompiledGp> elem);
        void visit(shared_ptr<CompiledLinSigmoid> elem);
        void visit(shared_ptr<CompiledLog> elem);
        void visit(shared_ptr<CompiledLTConstraint> elem);
        void visit(shared_ptr<CompiledLTEConstraint> elem);
        void visit(shared_ptr<CompiledMax> elem);
        void visit(shared_ptr<CompiledMin> elem);
        void visit(shared_ptr<CompiledOr> elem);
        void visit(shared_ptr<CompiledProduct> elem);
        void visit(shared_ptr<CompiledReification> elem);
        void visit(shared_ptr<CompiledSigmoid> elem);
        void visit(shared_ptr<CompiledSin> elem);
        void visit(shared_ptr<CompiledSum> elem);
        void visit(shared_ptr<CompiledTermPower> elem);
        void visit(shared_ptr<CompiledVariable> var);

    private:
        vector<shared_ptr<TapeElement>>* _tape;

        double valueOf(int index);
    };
};

} /* namespace autodiff */

#endif /* COMPILEDDIFFERENTIATOR_H_ */
