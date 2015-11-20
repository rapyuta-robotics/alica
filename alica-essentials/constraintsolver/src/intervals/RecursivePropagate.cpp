/*
 * RecursivePropagate.cpp
 *
 *  Created on: Dec 11, 2014
 *      Author: Philipp
 */

#include "intervals/RecursivePropagate.h"
#define USEQUEUE

#include "intervals/DownwardPropagator.h"
#include "intervals/SetParents.h"
#include "intervals/TermList.h"
#include "intervals/UpwardPropagator.h"

//#define RecPropDEBUG

namespace alica
{
	namespace reasoner
	{
		namespace intervalpropagation
		{

			RecursivePropagate::RecursivePropagate()
			{
				changed = make_shared<TermList>();

				dp = make_shared<DownwardPropagator>();
				dp->changed = this->changed;
				up = make_shared<UpwardPropagator>();
				up->changed = this->changed;
				sp = make_shared<SetParents>();
			}

			RecursivePropagate::~RecursivePropagate()
			{
				// TODO Auto-generated destructor stub
			}

			bool RecursivePropagate::propagate(shared_ptr<Term> term)
			{
				//for(int i=0; i<2; i++) {
				this->changed->clear();
				term->accept(shared_from_this());
#ifdef RecPropDEBUG
				cout << "Queued Terms: " << endl;
				shared_ptr<Term> asd = this->changed->first;
				while (asd != nullptr)
				{
					cout << asd->toString() << endl;
					asd = asd->next;
				}
				cout << "------------------------------------" << endl;
#endif
				/*
				 foreach(Term q in this->changed) {
				 q->accept(this->sp);
				 }
				 while(this->changed.Count > 0) {
				 Term cur = this->changed->dequeue();
				 cur->accept(this->dp);
				 cur->accept(this->up);
				 }*/

				shared_ptr<Term> cur = this->changed->first;
				while (cur != nullptr)
				{
					cur->accept(this->sp);
					cur = cur->next;
				}

				cur = this->changed->dequeue();
				while (cur != nullptr)
				{
					cur->accept(this->dp);
					cur->accept(this->up);
					cur = this->changed->dequeue();
				}

				//}
				/*cur = this->changed->first;
				 while(cur!=null) {
				 cur->accept(this->dp);
				 //Term next = cur->next;

				 if(cur->accept(this->up)) {
				 /*Term prev = cur.Prev;
				 this->changed.MoveToEnd(cur);
				 if (prev == nullptr) cur = this->changed->first;
				 else cur = prev->next;*/
				//cur = cur->next;
				//	}
				//	cur = cur->next;
				//}
				//*/
			}

			void RecursivePropagate::addToQueue(shared_ptr<Term> t)
			{
				if (!this->changed->contains(t))
					this->changed->enqueue(t);
			}

			int RecursivePropagate::visit(shared_ptr<Abs> abs)
			{
				addToQueue(abs);
				abs->arg->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<And> and_)
			{
				addToQueue(and_);
				and_->left->accept(shared_from_this());
				and_->right->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Atan2> atan2)
			{
				addToQueue(atan2);
				atan2->left->accept(shared_from_this());
				atan2->right->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Constant> constant)
			{
				//	return false;
			}

			int RecursivePropagate::visit(shared_ptr<ConstPower> intPower)
			{
				addToQueue(intPower);
				intPower->base->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<ConstraintUtility> cu)
			{
				addToQueue(cu);
				cu->constraint->accept(shared_from_this());
				cu->utility->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Cos> cos)
			{
				addToQueue(cos);
				cos->arg->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Exp> exp)
			{
				addToQueue(exp);
				exp->arg->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Gp> gp)
			{
				throw "Not implemented yet";
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<LinSigmoid> sigmoid)
			{
				addToQueue(sigmoid);
				sigmoid->arg->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Log> log)
			{
				addToQueue(log);
				log->arg->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<LTConstraint> constraint)
			{
				addToQueue(constraint);
				constraint->left->accept(shared_from_this());
				constraint->right->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<LTEConstraint> constraint)
			{
				addToQueue(constraint);
				constraint->left->accept(shared_from_this());
				constraint->right->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Max> max)
			{
				addToQueue(max);
				max->left->accept(shared_from_this());
				max->right->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Min> min)
			{
				addToQueue(min);
				min->left->accept(shared_from_this());
				min->right->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Or> or_)
			{
				addToQueue(or_);
				or_->left->accept(shared_from_this());
				or_->right->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Product> product)
			{
				addToQueue(product);
				product->left->accept(shared_from_this());
				product->right->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Reification> reif)
			{
				addToQueue(reif);
				reif->condition->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Sigmoid> sigmoid)
			{
				addToQueue(sigmoid);
				sigmoid->arg->accept(shared_from_this());
				sigmoid->mid->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Sin> sin)
			{
				addToQueue(sin);
				sin->arg->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Sum> sum)
			{
				addToQueue(sum);
				for (shared_ptr<Term> t : sum->terms)
				{
					t->accept(shared_from_this());
				}
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<TermPower> power)
			{
				addToQueue(power);
				power->base->accept(shared_from_this());
				power->exponent->accept(shared_from_this());
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Variable> var)
			{
				addToQueue(var);
				//return true;
			}

			int RecursivePropagate::visit(shared_ptr<Zero> zero)
			{
				//	return false;
			}

		} /* namespace intervalpropagation */
	} /* namespace reasoner */
} /* namespace alica */
