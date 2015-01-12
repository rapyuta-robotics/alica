/*
 * Lit.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#include "types/Lit.h"

#include "types/Var.h"

#include <iostream>

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{

			Lit::Lit(shared_ptr<Var> v, Assignment ass)
			{
				this->var = v;
				this->sign = ass;
				if (ass == Assignment::TRUE)
				{
					var->positiveAppearance++;
				}
				else
				{
					var->negActivity++;
				}
				this->isTemporary = false;
				variableCount = -1;
			}

			Lit::Lit(shared_ptr<Term> t, Assignment ass, bool temp)
			{
				this->isTemporary = temp;
				this->sign = ass;
				this->atom = t;
				variableCount = -1;
			}

			Lit::~Lit()
			{
				// TODO Auto-generated destructor stub
			}

			bool Lit::satisfied()
			{
				return sign == var->assignment;
			}

			bool Lit::conflicted()
			{
				return var->assignment != sign && var->assignment != Assignment::UNASSIGNED;
			}

			void Lit::computeVariableCount()
			{
				variableCount = 0;
				atom->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Abs> abs)
			{
				abs->arg->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<And> and_)
			{
				and_->left->accept(shared_from_this());
				and_->right->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Atan2> atan2)
			{
				atan2->left->accept(shared_from_this());
				atan2->right->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Constant> constant)
			{
			}

			int Lit::visit(shared_ptr<ConstPower> intPower)
			{
				intPower->base->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<ConstraintUtility> cu)
			{
				cu->constraint->accept(shared_from_this());
				cu->utility->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Cos> cos)
			{
				cos->arg->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Exp> exp)
			{
				exp->arg->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Gp> gp)
			{
				cerr << "Not Implemented Yet!" << endl;
				throw "Not Implemented Yet!";
			}

			int Lit::visit(shared_ptr<LinSigmoid> sigmoid)
			{
				sigmoid->arg->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Log> log)
			{
				log->arg->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<LTConstraint> constraint)
			{
				constraint->left->accept(shared_from_this());
				constraint->right->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<LTEConstraint> constraint)
			{
				constraint->left->accept(shared_from_this());
				constraint->right->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Max> max)
			{
				max->left->accept(shared_from_this());
				max->right->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Min> min)
			{
				min->left->accept(shared_from_this());
				min->right->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Or> or_)
			{
				or_->left->accept(shared_from_this());
				or_->right->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Product> product)
			{
				product->left->accept(shared_from_this());
				product->right->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Reification> dis)
			{
				dis->condition->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Sigmoid> sigmoid)
			{
				sigmoid->arg->accept(shared_from_this());
				sigmoid->mid->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Sin> sin)
			{
				sin->arg->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Sum> sum)
			{
				for (shared_ptr<Term> t : sum->terms)
				{
					t->accept(shared_from_this());
				}
			}

			int Lit::visit(shared_ptr<TermPower> power)
			{
				power->base->accept(shared_from_this());
				power->exponent->accept(shared_from_this());
			}

			int Lit::visit(shared_ptr<Variable> var)
			{
				variableCount++;
			}

			int Lit::visit(shared_ptr<Zero> zero)
			{
			}

		} /* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */
