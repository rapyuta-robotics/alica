/*
 * ResetIntervals.cpp
 *
 *  Created on: Dec 11, 2014
 *      Author: Philipp
 */

#include "intervals/ResetIntervals.h"

#include <limits>
#include <math.h>

namespace alica
{
	namespace reasoner
	{
		namespace intervalpropagation
		{

			ResetIntervals::ResetIntervals()
			{
			}

			ResetIntervals::~ResetIntervals()
			{
				// TODO Auto-generated destructor stub
			}

			int ResetIntervals::visit(shared_ptr<Abs> abs)
			{
				abs->parents.clear();
				abs->arg->accept(shared_from_this());
				updateInterval(abs, 0, numeric_limits<double>::infinity());
				return true;
			}

			int ResetIntervals::visit(shared_ptr<And> and_)
			{
				and_->parents.clear();
				and_->left->accept(shared_from_this());
				and_->right->accept(shared_from_this());
				updateInterval(and_, -numeric_limits<double>::infinity(), 1);
				//updateInterval(and,1,1); //enforce the purely conjunctive problem
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Atan2> atan2)
			{
				atan2->parents.clear();
				atan2->left->accept(shared_from_this());
				atan2->right->accept(shared_from_this());
				updateInterval(atan2, -M_PI, M_PI);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Constant> constant)
			{
				constant->parents.clear();
				updateInterval(constant, constant->value, constant->value);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<ConstPower> intPower)
			{
				intPower->parents.clear();

				intPower->base->accept(shared_from_this());

				if (intPower->exponent == 0)
				{
					updateInterval(intPower, 0, 0);
					return true;
				}
				double e = round(intPower->exponent);
				if (intPower->exponent == e && ((int)e) % 2 == 0)
				{
					updateInterval(intPower, 0, numeric_limits<double>::infinity());
					return true;
				}
				updateInterval(intPower, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
				return false;
			}

			int ResetIntervals::visit(shared_ptr<ConstraintUtility> cu)
			{
				cu->parents.clear();
				cu->constraint->accept(shared_from_this());
				cu->utility->accept(shared_from_this());
				updateInterval(cu, 1, numeric_limits<double>::infinity());
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Cos> cos)
			{
				cos->parents.clear();
				cos->arg->accept(shared_from_this());
				updateInterval(cos, -1, 1);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Exp> exp)
			{
				exp->parents.clear();
				exp->arg->accept(shared_from_this());
				updateInterval(exp, 0, numeric_limits<double>::infinity());
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Gp> gp)
			{
				throw "Not implemented yet";
				return false;
			}

			int ResetIntervals::visit(shared_ptr<LinSigmoid> sigmoid)
			{
				sigmoid->parents.clear();
				sigmoid->arg->accept(shared_from_this());
				updateInterval(sigmoid, 0, 1);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Log> log)
			{
				log->parents.clear();
				log->arg->accept(shared_from_this());
				updateInterval(log, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
				return true;
			}

			int ResetIntervals::visit(shared_ptr<LTConstraint> constraint)
			{
				constraint->parents.clear();
				constraint->left->accept(shared_from_this());
				constraint->right->accept(shared_from_this());
				updateInterval(constraint, -numeric_limits<double>::infinity(), 1);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<LTEConstraint> constraint)
			{
				constraint->parents.clear();
				constraint->left->accept(shared_from_this());
				constraint->right->accept(shared_from_this());
				updateInterval(constraint, -numeric_limits<double>::infinity(), 1);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Max> max)
			{
				max->parents.clear();
				max->left->accept(shared_from_this());
				max->right->accept(shared_from_this());
				updateInterval(max, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Min> min)
			{
				min->parents.clear();
				min->left->accept(shared_from_this());
				min->right->accept(shared_from_this());
				updateInterval(min, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Or> or_)
			{
				or_->parents.clear();
				or_->left->accept(shared_from_this());
				or_->right->accept(shared_from_this());
				updateInterval(or_, -numeric_limits<double>::infinity(), 1);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Product> product)
			{
				product->parents.clear();
				updateInterval(product, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
				product->left->accept(shared_from_this());
				product->right->accept(shared_from_this());
				return false;
			}

			int ResetIntervals::visit(shared_ptr<Reification> reif)
			{
				reif->parents.clear();
				reif->condition->accept(shared_from_this());
				updateInterval(reif, reif->min, reif->max);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Sigmoid> sigmoid)
			{
				sigmoid->parents.clear();
				sigmoid->arg->accept(shared_from_this());
				sigmoid->mid->accept(shared_from_this());
				updateInterval(sigmoid, 0, 1);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Sin> sin)
			{
				sin->parents.clear();
				sin->arg->accept(shared_from_this());
				updateInterval(sin, -1, 1);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Sum> sum)
			{
			}

			int ResetIntervals::visit(shared_ptr<TermPower> power)
			{
				power->parents.clear();
				power->base->accept(shared_from_this());
				power->exponent->accept(shared_from_this());
				updateInterval(power, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
				return false;
			}

			int ResetIntervals::visit(shared_ptr<Variable> var)
			{
				var->parents.clear();
				updateInterval(var, var->globalMin, var->globalMax);
				return true;
			}

			int ResetIntervals::visit(shared_ptr<Zero> zero)
			{
				zero->parents.clear();
				updateInterval(zero, 0, 0);
				return true;
			}

			void ResetIntervals::updateInterval(shared_ptr<Term> t, double min, double max)
			{
				t->min = min;
				t->max = max;
				return;
			}

		} /* namespace intervalpropagation */
	} /* namespace reasoner */
} /* namespace alica */
