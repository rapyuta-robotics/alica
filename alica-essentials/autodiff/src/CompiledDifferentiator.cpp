/*
 * CompiledDifferentiator.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#include "CompiledDifferentiator.h"

#include "Term.h"
#include "TermBuilder.h"
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
#include "compiled/CompiledAbs.h"
#include "compiled/CompiledAnd.h"
#include "compiled/CompiledAtan2.h"
#include "compiled/CompiledConstant.h"
#include "compiled/CompiledConstPower.h"
#include "compiled/CompiledConstraintUtility.h"
#include "compiled/CompiledCos.h"
#include "compiled/CompiledExp.h"
#include "compiled/CompiledGp.h"
#include "compiled/CompiledLinSigmoid.h"
#include "compiled/CompiledLog.h"
#include "compiled/CompiledLTConstraint.h"
#include "compiled/CompiledLTEConstraint.h"
#include "compiled/CompiledMax.h"
#include "compiled/CompiledMin.h"
#include "compiled/CompiledOr.h"
#include "compiled/CompiledProduct.h"
#include "compiled/CompiledReification.h"
#include "compiled/CompiledSigmoid.h"
#include "compiled/CompiledSin.h"
#include "compiled/CompiledSum.h"
#include "compiled/CompiledTermPower.h"
#include "compiled/CompiledVariable.h"

#include <cmath>
#include <limits>
#include <algorithm>

#include <iostream>

namespace autodiff
{
	CompiledDifferentiator::CompiledDifferentiator(shared_ptr<Term> function, shared_ptr<vector<shared_ptr<Variable>>> variables)
	{
		if (dynamic_pointer_cast<Variable>(function) != 0)
		{
			function = make_shared<ConstPower>(function, 1);
		}

		vector<shared_ptr<TapeElement>> tapeList;
		make_shared<Compiler>(variables, &tapeList)->compile(function);
		_tape = tapeList;

		_dimension = variables->size();
		_variables = variables;
	}

	double CompiledDifferentiator::evaluate(shared_ptr<vector<double>> arg)
	{
		evaluateTape(arg);
		return _tape.back()->value;
	}

	pair<shared_ptr<vector<double>>, double> CompiledDifferentiator::differentiate(shared_ptr<vector<double>> arg)
	{
		forwardSweep(arg);
		reverseSweep();
		//Replacement for Linq code -- HS
		auto gradient = make_shared<vector<double>>(_dimension);

		for (int i = 0; i < _dimension; ++i)
		{
			gradient->at(i) = _tape[i]->adjoint;
		}
		double value = _tape[_tape.size() - 1]->value;

		return pair<shared_ptr<vector<double>>, double>(gradient, value);
	}

	void CompiledDifferentiator::forwardSweep(shared_ptr<vector<double>> arg)
	{
		for (int i = 0; i < _dimension; ++i)
		{
			_tape[i]->value = arg->at(i);
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
			vector<InputEdge> inputs = _tape[i]->inputs;
			double adjoint = _tape[i]->adjoint;

			for (int j = 0; j < inputs.size(); ++j)
			{
				_tape[inputs[j].index]->adjoint += adjoint * inputs[j].weight;
			}
		}
	}

	void CompiledDifferentiator::evaluateTape(shared_ptr<vector<double>> arg)
	{
		for (int i = 0; i < _dimension; ++i)
		{
			_tape[i]->value = arg->at(i);
		}

		shared_ptr<EvalVisitor> evalVisitor = make_shared<EvalVisitor>(&_tape);
		for (int i = _dimension; i < _tape.size(); ++i)
		{
			_tape[i]->accept(evalVisitor);
		}
	}

	CompiledDifferentiator::Compiler::Compiler(shared_ptr<vector<shared_ptr<Variable>>> variables,
												vector<shared_ptr<TapeElement>>* tape)
	{
		_tape = tape;
		for (int i = 0; i < variables->size(); ++i)
		{
			_indexOf[variables->at(i)->getIndex()] = i;
			tape->push_back(make_shared<CompiledVariable>());
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
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(abs, [&abs, &storedThis]()
		{
			int argIndex = abs->getArg()->accept(storedThis);

			shared_ptr<CompiledAbs> element = make_shared<CompiledAbs>();
			element->_arg = argIndex;
			element->inputs = vector<InputEdge>(1);
			element->inputs[0].index = argIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<And> and_)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(and_, [&and_, &storedThis]()
		{
			int leftIndex = and_->getLeft()->accept(storedThis);
			int rightIndex = and_->getRight()->accept(storedThis);

			shared_ptr<CompiledAnd> element = make_shared<CompiledAnd>();
			element->_left = leftIndex;
			element->_right = rightIndex;
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = leftIndex;
			element->inputs[1].index = rightIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Atan2> atan2)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(atan2, [&atan2, &storedThis]()
		{
			int leftIndex = atan2->getLeft()->accept(storedThis);
			int rightIndex = atan2->getRight()->accept(storedThis);

			shared_ptr<CompiledAtan2> element = make_shared<CompiledAtan2>();
			element->_left = leftIndex;
			element->_right = rightIndex;
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = leftIndex;
			element->inputs[1].index = rightIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Constant> constant)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(
				constant,
				[&constant, &storedThis]()
				{
					shared_ptr<CompiledConstant> element = make_shared<CompiledConstant>(constant->getValue());
					return element;
				});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<ConstPower> intPower)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(intPower, [&intPower, &storedThis]()
		{
			int baseIndex = intPower->getBase()->accept(storedThis);

			shared_ptr<CompiledConstPower> element = make_shared<CompiledConstPower>();
			element->_base = baseIndex;
			element->_exponent = intPower->getExponent();
			element->inputs = vector<InputEdge>(1);
			element->inputs[0].index = baseIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<ConstraintUtility> cu)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(
				cu,
				[&cu, &storedThis]()
				{
					int constraintIndex = cu->getConstraint()->accept(storedThis);
					int utilityIndex = cu->getUtility()->accept(storedThis);

					shared_ptr<CompiledConstraintUtility> element = make_shared<CompiledConstraintUtility>();
					element->_constraint = constraintIndex;
					element->_utility = utilityIndex;
					element->inputs = vector<InputEdge>(2);
					element->inputs[0].index = constraintIndex;
					element->inputs[1].index = utilityIndex;

					return element;
				});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Cos> cos)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(cos, [&cos, &storedThis]()
		{
			int argIndex = cos->getArg()->accept(storedThis);

			shared_ptr<CompiledCos> element = make_shared<CompiledCos>();
			element->_arg = argIndex;
			element->inputs = vector<InputEdge>(1);
			element->inputs[0].index = argIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Exp> exp)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(exp, [&exp, &storedThis]()
		{
			int argIndex = exp->getArg()->accept(storedThis);

			shared_ptr<CompiledExp> element = make_shared<CompiledExp>();
			element->_arg = argIndex;
			element->inputs = vector<InputEdge>(1);
			element->inputs[0].index = argIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Gp> gp)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::Compiler::visit(shared_ptr<Gp> gp)" << endl;
		throw "NOT IMPLEMENTED YET";
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<LinSigmoid> sigmoid)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(sigmoid, [&sigmoid, &storedThis]()
		{
			int argIndex = sigmoid->getArg()->accept(storedThis);

			//XXX: warum nich CompiledLinSigmoid?
						shared_ptr<CompiledSigmoid> element = make_shared<CompiledSigmoid>();
						element->_arg = argIndex;
						element->inputs = vector<InputEdge>(2);
						element->inputs[0].index = argIndex;

						return element;
					});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Log> log)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(log, [&log, &storedThis]()
		{
			int argIndex = log->getArg()->accept(storedThis);

			shared_ptr<CompiledLog> element = make_shared<CompiledLog>();
			element->_arg = argIndex;
			element->inputs = vector<InputEdge>(1);
			element->inputs[0].index = argIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<LTConstraint> constraint)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(constraint, [&constraint, &storedThis]()
		{
			int leftIndex = constraint->getLeft()->accept(storedThis);
			int rightIndex = constraint->getRight()->accept(storedThis);

			shared_ptr<CompiledLTConstraint> element = make_shared<CompiledLTConstraint>();
			element->_left = leftIndex;
			element->_right = rightIndex;
			element->_steepness = constraint->getSteppness();
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = leftIndex;
			element->inputs[1].index = rightIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<LTEConstraint> constraint)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(constraint, [&constraint, &storedThis]()
		{
			int leftIndex = constraint->getLeft()->accept(storedThis);
			int rightIndex = constraint->getRight()->accept(storedThis);

			shared_ptr<CompiledLTEConstraint> element = make_shared<CompiledLTEConstraint>();
			element->_left = leftIndex;
			element->_right = rightIndex;
			element->_steepness = constraint->getSteppness();
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = leftIndex;
			element->inputs[1].index = rightIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Max> max)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(max, [&max, &storedThis]()
		{
			int leftIndex = max->getLeft()->accept(storedThis);
			int rightIndex = max->getRight()->accept(storedThis);

			shared_ptr<CompiledMax> element = make_shared<CompiledMax>();
			element->_left = leftIndex;
			element->_right = rightIndex;
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = leftIndex;
			element->inputs[1].index = rightIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Min> min)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(min, [&min, &storedThis]()
		{
			int leftIndex = min->getLeft()->accept(storedThis);
			int rightIndex = min->getRight()->accept(storedThis);

			shared_ptr<CompiledMin> element = make_shared<CompiledMin>();
			element->_left = leftIndex;
			element->_right = rightIndex;
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = leftIndex;
			element->inputs[1].index = rightIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Or> or_)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(or_, [&or_, &storedThis]()
		{
			int leftIndex = or_->getLeft()->accept(storedThis);
			int rightIndex = or_->getRight()->accept(storedThis);

			shared_ptr<CompiledOr> element = make_shared<CompiledOr>();
			element->_left = leftIndex;
			element->_right = rightIndex;
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = leftIndex;
			element->inputs[1].index = rightIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Product> product)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(product, [&product, &storedThis]()
		{
			int leftIndex = product->getLeft()->accept(storedThis);
			int rightIndex = product->getRight()->accept(storedThis);

			shared_ptr<CompiledProduct> element = make_shared<CompiledProduct>();
			element->_left = leftIndex;
			element->_right = rightIndex;
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = leftIndex;
			element->inputs[1].index = rightIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Reification> dis)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(dis, [&dis, &storedThis]()
		{
			int conIndex = dis->getCondition()->accept(storedThis);
			int negConIndex = dis->getNegatedCondition()->accept(storedThis);

			shared_ptr<CompiledReification> element = make_shared<CompiledReification>();
			element->_min = dis->getMin();
			element->_max = dis->getMax();
			element->_condition = conIndex;
			element->_negatedCondition = negConIndex;
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = conIndex;
			element->inputs[1].index = negConIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Sigmoid> sigmoid)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(sigmoid, [&sigmoid, &storedThis]()
		{
			int argIndex = sigmoid->getArg()->accept(storedThis);
			int midIndex = sigmoid->getMid()->accept(storedThis);

			shared_ptr<CompiledSigmoid> element = make_shared<CompiledSigmoid>();
			element->_arg = argIndex;
			element->_mid= midIndex;
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = argIndex;
			element->inputs[1].index = midIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Sin> sin)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(sin, [&sin, &storedThis]()
		{
			int argIndex = sin->getArg()->accept(storedThis);
			shared_ptr<CompiledSin> element = make_shared<CompiledSin>();
			element->_arg = argIndex;
			element->inputs = vector<InputEdge>(1);
			element->inputs[0].index = argIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Sum> sum)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(sum, [&sum, &storedThis]()
		{
			//replacement for linq code -- HS:
						vector<int> indices(sum->getTerms().size());
						vector<InputEdge> inputs(indices.size());
						for (int i = 0; i < indices.size(); ++i)
						{
							indices[i] = sum->getTerms()[i]->accept(storedThis);
							inputs[i].index = indices[i];
						}

						shared_ptr<CompiledSum> element = make_shared<CompiledSum>();
						element->_terms = indices;
						element->inputs = inputs;

						return element;
					});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<TermPower> power)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(power, [&power, &storedThis]()
		{
			int baseIndex = power->getBase()->accept(storedThis);
			int expIndex = power->getExponent()->accept(storedThis);

			shared_ptr<CompiledTermPower> element = make_shared<CompiledTermPower>();
			element->_base = baseIndex;
			element->_exponent = expIndex;
			element->inputs = vector<InputEdge>(2);
			element->inputs[0].index = baseIndex;
			element->inputs[1].index = expIndex;

			return element;
		});
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Variable> var)
	{
		return _indexOf[var->getIndex()];
	}

	int CompiledDifferentiator::Compiler::visit(shared_ptr<Zero> zero)
	{
		shared_ptr<ITermVisitor> storedThis = shared_from_this();
		return compile(zero, [&zero, &storedThis]()
		{
			shared_ptr<CompiledConstant> element = make_shared<CompiledConstant>(0);
			return element;
		});
	}

	int CompiledDifferentiator::Compiler::compile(shared_ptr<Term> term,
													function<shared_ptr<TapeElement>()> compiler)
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
			shared_ptr<TapeElement> compileResult = compiler();
			_tape->push_back(compileResult);

			index = _tape->size() - 1;
			_indexOf.insert(pair<int, int>(term->getIndex(), index));
		}
		return index;
	}

	CompiledDifferentiator::EvalVisitor::EvalVisitor(vector<shared_ptr<TapeElement>> *tape)
	{
		_tape = tape;
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledAbs> elem)
	{
		elem->value = fabs(valueOf(elem->_arg));
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledAnd> elem)
	{
		if (valueOf(elem->_left) > 0.75)
		{
			elem->value = valueOf(elem->_right);
		}
		else if (valueOf(elem->_right) > 0.75)
		{
			elem->value = valueOf(elem->_left);
		}
		else
		{
			elem->value = valueOf(elem->_left) + valueOf(elem->_right);
		}
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledAtan2> elem)
	{
		elem->value = atan2(valueOf(elem->_left), valueOf(elem->_right));
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledConstant> elem)
	{

	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledConstPower> elem)
	{
		elem->value = pow(valueOf(elem->_base), elem->_exponent);
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledConstraintUtility> elem)
	{
		if (valueOf(elem->_constraint) < 0.999)
		{
			elem->value = valueOf(elem->_constraint);
		}
		else
		{
			elem->value = valueOf(elem->_constraint) * valueOf(elem->_utility);
		}
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledCos> elem)
	{
		elem->value = cos(valueOf(elem->_arg));
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledExp> elem)
	{
		elem->value = exp(valueOf(elem->_arg));
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledGp> elem)
	{
		cerr << "NOT IMPLEMENTED: CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledGp> elem)"
				<< endl;
		throw "NOT IMPLEMENTED YET";
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledLinSigmoid> elem)
	{
		double e = exp(-valueOf(elem->_arg));
		if (e == numeric_limits<double>::infinity())
		{
			elem->value = Term::EPSILON;
		}
		else
		{
			elem->value = 1.0 / (1.0 + e);
		}
		if (elem->value < Term::EPSILON)
		{
			elem->value = Term::EPSILON;
		}
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledLog> elem)
	{
		elem->value = log(valueOf(elem->_arg));
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledLTConstraint> elem)
	{
		if (valueOf(elem->_left) < valueOf(elem->_right))
		{
			elem->value = 1;
		}
		else
		{
			elem->value = elem->_steepness * (valueOf(elem->_right) - valueOf(elem->_left));
		}
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledLTEConstraint> elem)
	{
		if (valueOf(elem->_left) < valueOf(elem->_right))
		{
			elem->value = 1;
		}
		else
		{
			elem->value = elem->_steepness * (valueOf(elem->_right) - valueOf(elem->_left));
		}
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledMax> elem)
	{
		elem->value = max(valueOf(elem->_left), valueOf(elem->_right));
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledMin> elem)
	{
		elem->value = min(valueOf(elem->_left), valueOf(elem->_right));
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledOr> elem)
	{
		if (valueOf(elem->_left) > 0.75)
		{
			elem->value = valueOf(elem->_left);
		}
		else if (valueOf(elem->_right) > 0.75)
		{
			elem->value = valueOf(elem->_right);
		}
		else
		{
			elem->value = valueOf(elem->_left) + valueOf(elem->_right);
		}
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledProduct> elem)
	{
		elem->value = valueOf(elem->_left) * valueOf(elem->_right);
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledReification> elem)
	{
		if (valueOf(elem->_condition) > 0)
		{
			elem->value = elem->_max;
		}
		else
		{
			elem->value = elem->_min;
		}
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledSigmoid> elem)
	{
		double e = exp(elem->_steepness * (-valueOf(elem->_arg) + valueOf(elem->_mid)));
		if (e == numeric_limits<double>::infinity())
		{
			elem->value = Term::EPSILON;
		}
		else
		{
			elem->value = 1.0 / (1.0 + e);
		}
		if (elem->value < Term::EPSILON)
		{
			elem->value = Term::EPSILON;
		}
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledSin> elem)
	{
		elem->value = sin(valueOf(elem->_arg));
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledSum> elem)
	{
		elem->value = 0;
		for (int i = 0; i < elem->_terms.size(); ++i)
		{
			elem->value += valueOf(elem->_terms[i]);
		}
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledTermPower> elem)
	{
		elem->value = pow(valueOf(elem->_base), valueOf(elem->_exponent));
	}

	void CompiledDifferentiator::EvalVisitor::visit(shared_ptr<CompiledVariable> var)
	{

	}

	double CompiledDifferentiator::EvalVisitor::valueOf(int index)
	{
		return (*_tape)[index]->value;
	}

	CompiledDifferentiator::ForwardSweepVisitor::ForwardSweepVisitor(vector<shared_ptr<TapeElement>> *tape)
	{
		_tape = tape;
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledAbs> elem)
	{
		double arg = valueOf(elem->_arg);
		elem->value = fabs(arg);
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledAnd> elem)
	{
		double left = valueOf(elem->_left);
		double right = valueOf(elem->_right);
		if (left > 0.75 && right > 0.75)
		{
			elem->value = 1;
			elem->inputs[0].weight = 0;
			elem->inputs[1].weight = 0;
			return;
		}
		elem->value = 0;
		if (left <= 0)
		{
			elem->value += left;
			elem->inputs[0].weight = 1;
		}
		if (right <= 0)
		{
			elem->value += right;
			elem->inputs[1].weight = 1;
		}
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledAtan2> elem)
	{
		double left = valueOf(elem->_left);
		double right = valueOf(elem->_right);

		elem->value = atan2(left, right);
		double denom = left * left + right * right;
		elem->inputs[0].weight = -right / denom;
		elem->inputs[1].weight = left / denom;
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledConstant> elem)
	{

	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledConstPower> elem)
	{
		double baseVal = valueOf(elem->_base);
		//modified to remove one Math.Pow -- HS
		double r = pow(baseVal, elem->_exponent - 1);
		elem->value = r * baseVal;
		elem->inputs[0].weight = elem->_exponent * r;
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledConstraintUtility> elem)
	{
		double constraint = valueOf(elem->_constraint);
		if (constraint > 0)
		{
			elem->value = valueOf(elem->_utility);
			elem->inputs[0].weight = 0;
			elem->inputs[1].weight = 1;
		}
		else
		{
			elem->value = constraint;
			elem->inputs[0].weight = 1;
			elem->inputs[1].weight = 0;
		}
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledCos> elem)
	{
		double arg = valueOf(elem->_arg);

		elem->value = cos(arg);
		elem->inputs[0].weight = -sin(arg);
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledExp> elem)
	{
		elem->value = exp(valueOf(elem->_arg));
		elem->inputs[0].weight = elem->value;
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledGp> elem)
	{
		cerr
				<< "NOT IMPLEMENTED: CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledGp> elem)"
				<< endl;
		throw "NOT IMPLEMENTED YET";
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledLinSigmoid> elem)
	{
		double arg = valueOf(elem->_arg);
		double e = exp(-arg);

		if (e == numeric_limits<double>::infinity())
		{
			elem->value = Term::EPSILON;
		}
		else
		{
			elem->value = 1.0 / (1.0 + e);
		}
		if (elem->value < Term::EPSILON)
		{
			elem->value = Term::EPSILON;
		}
		if (e == 0.0 || e == numeric_limits<double>::infinity())
		{
			elem->inputs[0].weight = Term::EPSILON;
			elem->inputs[1].weight = -Term::EPSILON;
			return;
		}
		double e2 = e / ((e + 1) * (e + 1));
		elem->inputs[0].weight = e2;
		elem->inputs[1].weight = -e2;
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledLog> elem)
	{
		double arg = valueOf(elem->_arg);

		elem->value = log(arg);
		elem->inputs[0].weight = 1 / arg;
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledLTConstraint> elem)
	{
		double left = valueOf(elem->_left);
		double right = valueOf(elem->_right);

		if (left < right)
		{
			elem->value = 1;
			elem->inputs[0].weight = 0;
			elem->inputs[1].weight = 0;
		}
		else
		{
			elem->value = elem->_steepness * (right - left);
			elem->inputs[0].weight = -elem->_steepness;
			elem->inputs[1].weight = elem->_steepness;
		}
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledLTEConstraint> elem)
	{
		double left = valueOf(elem->_left);
		double right = valueOf(elem->_right);

		if (left <= right)
		{
			elem->value = 1;
			elem->inputs[0].weight = 0;
			elem->inputs[1].weight = 0;
		}
		else
		{
			elem->value = elem->_steepness * (right - left);
			elem->inputs[0].weight = -elem->_steepness;
			elem->inputs[1].weight = elem->_steepness;
		}
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledMax> elem)
	{
		double left = valueOf(elem->_left);
		double right = valueOf(elem->_right);

		elem->value = max(left, right);
		if (left > right)
		{
			elem->inputs[0].weight = 1;
			elem->inputs[1].weight = 0;
		}
		else
		{
			elem->inputs[0].weight = 0;
			elem->inputs[1].weight = 1;
		}
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledMin> elem)
	{
		double left = valueOf(elem->_left);
		double right = valueOf(elem->_right);

		elem->value = min(left, right);
		if (left < right)
		{
			elem->inputs[0].weight = 1;
			elem->inputs[1].weight = 0;
		}
		else
		{
			elem->inputs[0].weight = 0;
			elem->inputs[1].weight = 1;
		}
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledOr> elem)
	{
		double left = valueOf(elem->_left);
		double right = valueOf(elem->_right);

		if (left > 0.75 || right > 0.75)
		{
			elem->value = 1;
			elem->inputs[0].weight = 0;
			elem->inputs[1].weight = 0;
			return;
		}
		elem->value = 0;
		if (left <= 0)
		{
			elem->value += left;
			elem->inputs[0].weight = 1;
		}
		else
		{
			elem->inputs[0].weight = 0;
		}
		if (right <= 0)
		{
			elem->value += right;
			elem->inputs[1].weight = 1;
		}
		else
		{
			elem->inputs[1].weight = 0;
		}
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledProduct> elem)
	{
		double left = valueOf(elem->_left);
		double right = valueOf(elem->_right);

		elem->value = left * right;
		elem->inputs[0].weight = right;
		elem->inputs[1].weight = left;
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledReification> elem)
	{
		double condition = valueOf(elem->_condition);
		double d = elem->_max - elem->_min;

		if (condition > 0)
		{
			elem->value = elem->_max;
			elem->inputs[0].weight = 0;
			elem->inputs[1].weight = -d;
		}
		else
		{
			elem->value = elem->_min;
			elem->inputs[0].weight = d;
			elem->inputs[1].weight = 0;
		}
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledSigmoid> elem)
	{
		double arg = valueOf(elem->_arg);
		double mid = valueOf(elem->_mid);
		double e = exp(elem->_steepness * (-arg + mid));

		if (e == numeric_limits<double>::infinity())
		{
			elem->value = Term::EPSILON;
		}
		else
		{
			elem->value = 1.0 / (1.0 + e);
		}
		if (elem->value < Term::EPSILON)
		{
			elem->value = Term::EPSILON;
		}
		if (e == 0.0 || e == numeric_limits<double>::infinity())
		{
			elem->inputs[0].weight = elem->_steepness * Term::EPSILON;
			elem->inputs[1].weight = -elem->_steepness * Term::EPSILON;
			return;
		}
		double e2 = elem->_steepness * e / ((e + 1) * (e + 1));
		elem->inputs[0].weight = e2;
		elem->inputs[1].weight = -e2;
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledSin> elem)
	{
		double arg = valueOf(elem->_arg);

		elem->value = sin(arg);
		elem->inputs[0].weight = cos(arg);
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledSum> elem)
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

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledTermPower> elem)
	{
		double baseVal = valueOf(elem->_base);
		double exponent = valueOf(elem->_exponent);

		elem->value = pow(baseVal, exponent);
		elem->inputs[0].weight = exponent * pow(baseVal, exponent - 1);
		elem->inputs[1].weight = elem->value * log(baseVal);
	}

	void CompiledDifferentiator::ForwardSweepVisitor::visit(shared_ptr<CompiledVariable> var)
	{

	}

	double CompiledDifferentiator::ForwardSweepVisitor::valueOf(int index)
	{
		return (*_tape)[index]->value;
	}
} /* namespace autodiff */
