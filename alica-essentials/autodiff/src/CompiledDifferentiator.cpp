/*
 * CompiledDifferentiator.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#include "CompiledDifferentiator.h"

#include "Term.h"
#include "Abs.h"
#include "And.h"
#include "Atan2.h"
#include "Constant.h"
#include "ConstPower.h"
#include "ConstraintUtility.h"
#include "Cos.h"
#include "Exp.h"
#include "Gp.h"
#include "LinSigmoid.h"
#include "Log.h"
#include "LTConstraint.h"
#include "LTEConstraint.h"
#include "Max.h"
#include "Min.h"
#include "Or.h"
#include "Product.h"
#include "Reification.h"
#include "Sigmoid.h"
#include "Sin.h"
#include "Sum.h"
#include "TermPower.h"
#include "Variable.h"
#include "Zero.h"
#include "compiled/CompiledConstant.h"
#include "compiled/CompiledExp.h"
#include "compiled/CompiledProduct.h"
#include "compiled/CompiledSum.h"
#include "compiled/CompiledVariable.h"

#include <math.h>

#include <iostream>

namespace AutoDiff
{
	CompiledDifferentiator::CompiledDifferentiator(shared_ptr<Term> function, vector<shared_ptr<Variable>> variables)
	{
		if (dynamic_pointer_cast<Variable>(function) != 0)
		{
			function = make_shared<ConstPower>(function, 1);
		}

		vector<shared_ptr<Compiled::TapeElement>> tapeList;
		make_shared<Compiler>(variables, &tapeList)->compile(function);
		_tape = tapeList;

		_dimension = variables.size();
		_variables = variables;
	}

	double CompiledDifferentiator::evaluate(vector<double> arg)
	{
		evaluateTape(arg);
		return _tape.back()->value;
	}

	pair<vector<double>, double> CompiledDifferentiator::differentiate(vector<double> arg)
	{
		forwardSweep(arg);
		reverseSweep();
		//Replacement for Linq code -- HS
		vector<double> gradient(_dimension);

		for (int i = 0; i < _dimension; ++i)
		{
			gradient[i] = _tape[i]->adjoint;
		}
		double value = _tape[_tape.size() - 1]->value;

		return pair<vector<double>, double>(gradient, value);
	}

	void CompiledDifferentiator::forwardSweep(vector<double> arg)
	{
		for (int i = 0; i < _dimension; ++i)
		{
			_tape[i]->value = arg[i];
		}

		shared_ptr<ForwardSweepVisitor> forwardDiffVisitor = make_shared<ForwardSweepVisitor>(&_tape);
		for (int i = _dimension; i < _tape.size(); ++i)
		{
			_tape[i]->accept(forwardDiffVisitor);
		}
	}

	void CompiledDifferentiator::reverseSweep()
	{
		//Removed Linq code -- HS
		_tape[_tape.size() - 1]->adjoint = 1;

		// initialize adjoints
		for (int i = 0; i < _tape.size() - 1; ++i)
		{
			_tape[i]->adjoint = 0;
		}

		// accumulate adjoints
		for (int i = _tape.size() - 1; i >= _dimension; --i)
		{
			vector<Compiled::InputEdge> inputs = _tape[i]->inputs;
			double adjoint = _tape[i]->adjoint;

			for (int j = 0; j < inputs.size(); ++j)
			{
				_tape[inputs[j].index]->adjoint += adjoint * inputs[j].weight;
			}
		}
	}

	void CompiledDifferentiator::evaluateTape(vector<double> arg)
	{
		for (int i = 0; i < _dimension; ++i)
		{
			_tape[i]->value = arg[i];
		}

		shared_ptr<EvalVisitor> evalVisitor = make_shared<EvalVisitor>(&_tape);
		for (int i = _dimension; i < _tape.size(); ++i)
		{
			_tape[i]->accept(evalVisitor);
		}
	}

	CompiledDifferentiator::Compiler::Compiler(vector<shared_ptr<Variable>> variables,
												vector<shared_ptr<Compiled::TapeElement>>* tape)
	{
		_tape = tape;
		for (int i = 0; i < variables.size(); ++i)
		{
			_indexOf[variables[i]->getIndex()] = i;
			tape->push_back(make_shared<Compiled::CompiledVariable>());
		}
	}

	CompiledDifferentiator::Compiler::~Compiler()
	{

	}

	void CompiledDifferentiator::Compiler::compile(shared_ptr<Term> term)
	{
		term->accept(shared_from_this());
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Abs> abs)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<Abs> abs)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<And> and_)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<And> and_)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Atan2> atan2)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<Atan2> atan2)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Constant> constant)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(
				constant,
				[&constant, &storedThis]()
				{
					shared_ptr<Compiled::CompiledConstant> element = make_shared<Compiled::CompiledConstant>(constant->getValue());
					return element;
				});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<ConstPower> intPower)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<ConstPower> intPower)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<ConstraintUtility> cu)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptrr<ConstraintUtility> cu)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Cos> cos)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptrr<Cos> cos)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Exp> exp)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(exp, [&exp, &storedThis]()
		{
			int argIndex = exp->getArg()->accept(storedThis);

			shared_ptr<Compiled::CompiledExp> element = make_shared<Compiled::CompiledExp>();
			element->_arg = argIndex;
			element->inputs = vector<Compiled::InputEdge>(1);
			element->inputs[0].index = argIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Gp> gp)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptrr<Gp> gp)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<LinSigmoid> sigmoid)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptrr<LinSigmoid> sigmoid)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Log> log)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptrr<Log> log)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<LTConstraint> constraint)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptrr<LTConstraint> constraint)"
				<< endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<LTEConstraint> constraint)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptrr<LTEConstraint> constraint)"
				<< endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Max> max)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<Max> max)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Min> min)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<Min> min)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Or> or_)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<Or> or_)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Product> product)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(product, [&product, &storedThis]()
		{
			int leftIndex = product->getLeft()->accept(storedThis);
			int rightIndex = product->getRight()->accept(storedThis);

			shared_ptr<Compiled::CompiledProduct> element = make_shared<Compiled::CompiledProduct>();
			element->_left = leftIndex;
			element->_right = rightIndex;
			element->inputs = vector<Compiled::InputEdge>(2);
			element->inputs[0].index = leftIndex;
			element->inputs[1].index = rightIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Reification> dis)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<Reification> dis)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Sigmoid> sigmoid)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptrr<Sigmoid> sigmoid)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Sin> sin)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptrr<Sin> sin)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Sum> sum)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(sum, [&sum, &storedThis]()
		{
			//replacement for linq code -- HS:
						vector<int> indices(sum->getTerms().size());
						vector<Compiled::InputEdge> inputs(indices.size());
						for (int i = 0; i < indices.size(); ++i)
						{
							indices[i] = sum->getTerms()[i]->accept(storedThis);
							inputs[i].index = indices[i];
						}

						shared_ptr<Compiled::CompiledSum> element = make_shared<Compiled::CompiledSum>();
						element->_terms = indices;
						element->inputs = inputs;

						return element;
					});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<TermPower> power)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<TermPower> power)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Variable> var)
	{
		return _indexOf[var->getIndex()];
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Zero> zero)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<Zero> zero)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::compile(shared_ptr<Term> term,
													function<shared_ptr<Compiled::TapeElement>()> compiler)
	{
		int index;
		map<int, int>::iterator it = _indexOf.find(term->getIndex());
		if (it != _indexOf.end())
		{
			// already contains
			index = it->second;
		}
		else
		{
			// add it
			shared_ptr<Compiled::TapeElement> compileResult = compiler();
			_tape->push_back(compileResult);

			index = _tape->size() - 1;
			_indexOf.insert(pair<int, int>(term->getIndex(), index));
		}
		return index;
	}

	CompiledDifferentiator::EvalVisitor::EvalVisitor(vector<shared_ptr<Compiled::TapeElement>> *tape)
	{
		_tape = tape;
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<Compiled::CompiledConstant> elem)
	{

	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<Compiled::CompiledExp> elem)
	{
		elem->value = exp(valueOf(elem->_arg));
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<Compiled::CompiledProduct> elem)
	{
		elem->value = valueOf(elem->_left) * valueOf(elem->_right);
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<Compiled::CompiledSum> elem)
	{
		elem->value = 0;
		for (int i = 0; i < elem->_terms.size(); ++i)
		{
			elem->value += valueOf(elem->_terms[i]);
		}
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<Compiled::CompiledVariable> var)
	{

	}

	double CompiledDifferentiator::EvalVisitor::valueOf(int index)
	{
		return (*_tape)[index]->value;
	}

	CompiledDifferentiator::ForwardSweepVisitor::ForwardSweepVisitor(vector<shared_ptr<Compiled::TapeElement>> *tape)
	{
		_tape = tape;
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<Compiled::CompiledConstant> elem)
	{

	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<Compiled::CompiledExp> elem)
	{
		elem->value = exp(valueOf(elem->_arg));
		elem->inputs[0].weight = elem->value;
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<Compiled::CompiledProduct> elem)
	{
		double left = valueOf(elem->_left);
		double right = valueOf(elem->_right);

		elem->value = left * right;
		elem->inputs[0].weight = right;
		elem->inputs[1].weight = left;
	}
	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<Compiled::CompiledSum> elem)
	{
		elem->value = 0;
		for (int i = 0; i < elem->_terms.size(); ++i)
		{
			elem->value += valueOf(elem->_terms[i]);
		}

		for (int i = 0; i < elem->inputs.size(); ++i)
		{
			elem->inputs[i].weight = 1;
		}
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<Compiled::CompiledVariable> var)
	{

	}

	double CompiledDifferentiator::ForwardSweepVisitor::valueOf(int index)
	{
		return (*_tape)[index]->value;
	}
} /* namespace AutoDiff */
