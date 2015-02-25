/*
 * SetParents.cpp
 *
 *  Created on: Dec 12, 2014
 *      Author: Philipp
 */

#include "intervals/SetParents.h"

#include <vector>

namespace alica
{
	namespace reasoner
	{
		namespace intervalpropagation
		{

			SetParents::SetParents()
			{
			}

			SetParents::~SetParents()
			{
			}

			int SetParents::visit(shared_ptr<Abs> abs)
			{
				abs->arg->parents.push_back(abs);
				//abs->arg->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<And> and_)
			{
				and_->left->parents.push_back(and_);
				and_->right->parents.push_back(and_);
				//and_->left->accept(this);
				//and_->right->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Atan2> atan2)
			{
				atan2->left->parents.push_back(atan2);
				atan2->right->parents.push_back(atan2);
				//atan2->left->accept(this);
				//atan2->right->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Constant> constant)
			{
				return false;
				//return UpdateInterval(constant,constant.Value,constant.Value);
			}

			int SetParents::visit(shared_ptr<ConstPower> intPower)
			{
				intPower->base->parents.push_back(intPower);
				//intPower->base->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<ConstraintUtility> cu)
			{
				cu->constraint->parents.push_back(cu);
				cu->utility->parents.push_back(cu);
				//cu->constraint->accept(this);
				//cu->utility->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Cos> cos)
			{
				cos->arg->parents.push_back(cos);
				//cos->arg->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Exp> exp)
			{
				exp->arg->parents.push_back(exp);
				//exp->arg->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Gp> gp)
			{
				throw "Not implemented yet";
				return false;
			}

			int SetParents::visit(shared_ptr<LinSigmoid> sigmoid)
			{
				sigmoid->arg->parents.push_back(sigmoid);
				//sigmoid->arg->accept(this);
				//sigmoid->mid->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Log> log)
			{
				log->arg->parents.push_back(log);
				//log->arg->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<LTConstraint> constraint)
			{
				constraint->left->parents.push_back(constraint);
				constraint->right->parents.push_back(constraint);
				//constraint->left->accept(this);
				//constraint->right->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<LTEConstraint> constraint)
			{
				constraint->left->parents.push_back(constraint);
				constraint->right->parents.push_back(constraint);
				//constraint->left->accept(this);
				//constraint->right->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Max> max)
			{
				max->left->parents.push_back(max);
				max->right->parents.push_back(max);
				//max->left->accept(this);
				//max->right->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Min> min)
			{
				min->left->parents.push_back(min);
				min->right->parents.push_back(min);

				//min->left->accept(this);
				//min->right->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Or> or_)
			{
				or_->left->parents.push_back(or_);
				or_->right->parents.push_back(or_);
				//or_->left->accept(this);
				//or_->right->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Product> product)
			{
				product->left->parents.push_back(product);
				product->right->parents.push_back(product);

				//product->left->accept(this);
				//product->right->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Reification> reif)
			{
				reif->condition->parents.push_back(reif);
				//reif->condition->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Sigmoid> sigmoid)
			{
				sigmoid->arg->parents.push_back(sigmoid);
				sigmoid->mid->parents.push_back(sigmoid);
				//sigmoid->arg->accept(this);
				//sigmoid->mid->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Sin> sin)
			{
				sin->arg->parents.push_back(sin);
				//sin->arg->accept(this);
				return false;
			}

			int SetParents::visit(shared_ptr<Sum> sum)
			{
				for (shared_ptr<Term> t : sum->terms)
				{
					t->parents.push_back(sum);
					//t->accept(this);
				}
				return false;
			}

			int SetParents::visit(shared_ptr<TermPower> power)
			{
				power->base->parents.push_back(power);
				power->exponent->parents.push_back(power);
				return false;
			}

			int SetParents::visit(shared_ptr<Variable> var)
			{
				return false;
			}

			int SetParents::visit(shared_ptr<Zero> zero)
			{
				return false;
				//return UpdateInterval(zero,0,0);
			}

		} /* namespace intervalpropagation */
	} /* namespace reasoner */
} /* namespace alica */
