/*
 * Sigmoid.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Sigmoid.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "ConstPower.h"
#include "Exp.h"

#include <cmath>
#include <limits>

namespace autodiff
{

	Sigmoid::Sigmoid(shared_ptr<Term> arg, shared_ptr<Term> mid) :
			Term()
	{
		_arg = arg;
		_mid = mid;
		_steepness = 1;
	}

	Sigmoid::Sigmoid(shared_ptr<Term> arg, shared_ptr<Term> mid, double steppness) :
			Term()
	{
		_arg = arg;
		_mid = mid;
		_steepness = steppness;
	}

	int Sigmoid::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Sigmoid> thisCasted = dynamic_pointer_cast<Sigmoid>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Sigmoid::aggregateConstants()
	{
		_arg = _arg->aggregateConstants();
		_mid = _mid->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_arg) != 0 && dynamic_pointer_cast<Constant>(_mid) != 0)
		{
			shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(_arg);
			shared_ptr<Constant> mid = dynamic_pointer_cast<Constant>(_mid);
			double e = exp(_steepness * (-arg->getValue() + mid->getValue()));
			if (e == numeric_limits<double>::infinity())
			{
				return TermBuilder::constant(Term::EPSILON);
			}
			else
			{
				e = 1.0 / (1.0 + e);
			}
			if (e < Term::EPSILON)
			{
				return TermBuilder::constant(Term::EPSILON);
			}
			else
			{
				return TermBuilder::constant(e);
			}
		}
		else
		{
			return shared_from_this();
		}
	}

	shared_ptr<Term> Sigmoid::derivative(shared_ptr<Variable> v)
	{
		shared_ptr<Term> t = _steepness * (_arg->derivative(v) - _mid->derivative(v))
				* make_shared<Exp>(_steepness * (-1 * _arg + _mid));
		return t / make_shared<ConstPower>(make_shared<Exp>(_steepness * _arg) + make_shared<Exp>(_steepness * _mid), 2);
	}

	const shared_ptr<Term> Sigmoid::getArg()
	{
		return _arg;
	}

	const shared_ptr<Term> Sigmoid::getMid()
	{
		return _mid;
	}

	const double Sigmoid::getSteepness()
	{
		return _steepness;
	}
} /* namespace autodiff */
