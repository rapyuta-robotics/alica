/*
 * UpwardPropagator.cpp
 *
 *  Created on: Dec 12, 2014
 *      Author: Philipp
 */

#include "intervals/UpwardPropagator.h"
//#define DEBUG_UP

#include "intervals/IntervalPropagator.h"
#include "intervals/DownwardPropagator.h"
#include "intervals/TermList.h"
#include "intervals/UnsolveableException.h"

#include <math.h>
#include <cmath>
#include <limits>

#include <iostream>

namespace alica
{
	namespace reasoner
	{
		namespace intervalpropagation
		{

			UpwardPropagator::UpwardPropagator()
			{
			}

			UpwardPropagator::~UpwardPropagator()
			{
				// TODO Auto-generated destructor stub
			}

			int UpwardPropagator::visit(shared_ptr<Abs> abs)
			{
				bool containsZero = abs->arg->min * abs->arg->max <= 0;
				bool c = false;
				if (containsZero)
					c = updateInterval(abs, 0, std::max(std::abs(abs->arg->min), std::abs(abs->arg->max)));
				else
					c = updateInterval(abs, std::min(std::abs(abs->arg->min), std::abs(abs->arg->max)),
										std::max(std::abs(abs->arg->min), std::abs(abs->arg->max)));
				if (c)
					addChanged(abs);
				return c;
			}

			int UpwardPropagator::visit(shared_ptr<And> and_)
			{
				if (and_->left->min > 0 && and_->right->min > 0)
				{
					if (updateInterval(and_, 1, 1))
					{
						addChanged(and_);
						return true;
					}
				}
				else if (and_->left->max <= 0 || and_->right->max <= 0)
				{
					if (updateInterval(and_, -numeric_limits<double>::infinity(), 0))
					{
						addChanged(and_);
						return true;
					}
				}
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<Atan2> atan2)
			{
				throw "Atan2 prop not implemented!";
			}

			int UpwardPropagator::visit(shared_ptr<Constant> constant)
			{
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<ConstPower> intPower)
			{
				bool includesZero = intPower->base->min * intPower->base->max <= 0;
				if (intPower->exponent > 0)
				{
					double a = pow(intPower->base->max, intPower->exponent);
					double b = pow(intPower->base->min, intPower->exponent);
					if (includesZero)
					{
						if (updateInterval(intPower, std::min(0.0, std::min(a, b)), std::max(0.0, std::max(a, b))))
						{
							//if(updateInterval(intPower,std::min(0,pow(intPower->base->min,intPower->exponent)),std::max(0,pow(intPower->base->max,intPower->exponent)))) {
							addChanged(intPower);
							return true;
						}
					}
					else
					{
						if (updateInterval(intPower, std::min(a, b), std::max(a, b)))
						{
							//if(updateInterval(intPower,pow(intPower->base->min,intPower->exponent),pow(intPower->base->max,intPower->exponent))) {
							addChanged(intPower);
							return true;
						}
					}
				}
				else if (!includesZero)
				{
					double a = pow(intPower->base->max, intPower->exponent);
					double b = pow(intPower->base->min, intPower->exponent);

					//Console.WriteLine("Cur: {0} [{1} : {2}]",intPower,intPower->min,intPower->max);
					//Console.WriteLine("Base: [{0} : {1}]",intPower->base->min,intPower->base->max);

					if (updateInterval(intPower, std::min(a, b), std::max(a, b)))
					{
						//Console.WriteLine("From UW intpower {0}",intPower->exponent);
						//if(updateInterval(intPower,pow(intPower->base->max,intPower->exponent),pow(intPower->base->min,intPower->exponent))) {
						addChanged(intPower);
						return true;
					}
				} //else +- Infinity is possible
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<ConstraintUtility> cu)
			{
				if (cu->constraint->max < 1)
				{
					if (updateInterval(cu, -numeric_limits<double>::infinity(), cu->constraint->max))
					{
						addChanged(cu);
						return true;
					}
				}
				if (updateInterval(cu, -numeric_limits<double>::infinity(), cu->utility->max))
				{
					addChanged(cu);
					return true;
				}
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<Cos> cos)
			{
				double size = cos->arg->max - cos->arg->min;
				bool c = false;
				if (size <= 2 * M_PI)
				{
					double a = std::cos(cos->arg->max);
					double b = std::cos(cos->arg->min);
					double x = ceil(cos->arg->min / M_PI);
					double y = floor(cos->arg->max / M_PI);
					if (x == y)
					{ //single extrema
						if (((int)x) % 2 == 0)
						{ //maxima
							c = updateInterval(cos, std::min(a, b), 1);
						}
						else
						{ //minima
							c = updateInterval(cos, -1, std::max(a, b));
						}
					}
					else if (x > y)
					{ //no extrema
						c = updateInterval(cos, std::min(a, b), std::max(a, b));
					} //multiple extrema, don't update
				}
				if (c)
					addChanged(cos);
				return c;
			}

			int UpwardPropagator::visit(shared_ptr<Exp> exp)
			{
				if (updateInterval(exp, std::exp(exp->arg->min), std::exp(exp->arg->max)))
				{
					addChanged(exp);
					return true;
				}
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<Gp> gp)
			{
				throw "Propagation for Gp not implemented";
			}

			int UpwardPropagator::visit(shared_ptr<LinSigmoid> sigmoid)
			{
				throw "Sigmoidal propagation not implemented";
			}

			int UpwardPropagator::visit(shared_ptr<Log> log)
			{
				if (updateInterval(log, std::log(log->arg->min), std::log(log->arg->max)))
				{
					addChanged(log);
					return true;
				}
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<LTConstraint> constraint)
			{
				if (constraint->left->max < constraint->right->min)
				{
					if (updateInterval(constraint, 1, 1))
					{
						addChanged(constraint);
						return true;
					}
				}
				else if (constraint->left->min >= constraint->right->max)
				{
					//Console.WriteLine("LT UP negated: {0} {1}",constraint->left->min ,constraint->right->max);
					if (updateInterval(constraint, -numeric_limits<double>::infinity(), 0))
					{
						addChanged(constraint);
						return true;
					}
				}
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<LTEConstraint> constraint)
			{
				if (constraint->left->max <= constraint->right->min)
				{
					if (updateInterval(constraint, 1, 1))
					{
						addChanged(constraint);
						return true;
					}
				}
				else if (constraint->left->min > constraint->right->max)
				{
					if (updateInterval(constraint, -numeric_limits<double>::infinity(), 0))
					{
						addChanged(constraint);
						return true;
					}
				}

				return false;
			}

			int UpwardPropagator::visit(shared_ptr<Max> max)
			{
				if (updateInterval(max, std::min(max->left->min, max->right->min),
									std::max(max->left->max, max->right->max)))
				{
					addChanged(max);
					return true;
				}
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<Min> min)
			{
				if (updateInterval(min, std::min(min->left->min, min->right->min),
									std::max(min->left->max, min->right->max)))
				{
					addChanged(min);
					return true;
				}
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<Or> or_)
			{
				if (or_->left->min > 0 || or_->right->min > 0)
				{
					if (updateInterval(or_, 1, 1))
					{
						addChanged(or_);
						return true;
					}
				}
				else if (or_->left->max <= 0 && or_->right->max <= 0)
				{
					if (updateInterval(or_, -numeric_limits<double>::infinity(), 0))
					{
						addChanged(or_);
						return true;
					}
				}
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<Product> product)
			{
				double aa = product->left->min * product->right->min;
				double bb = product->left->max * product->right->max;
				double max;
				double min;
				if (product->left == product->right)
				{
					min = std::min(aa, bb);
					max = std::max(aa, bb);
					if (product->left->min * product->left->max <= 0)
						min = 0;
				}
				else
				{
					double ab = product->left->min * product->right->max;
					double ba = product->left->max * product->right->min;
					max = std::max(aa, std::max(ab, std::max(ba, bb)));
					min = std::min(aa, std::min(ab, std::min(ba, bb)));
				}
				if (updateInterval(product, min, max))
				{
					addChanged(product);
					return true;
				}
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<Reification> dis)
			{
			}

			int UpwardPropagator::visit(shared_ptr<Sigmoid> sigmoid)
			{
				throw "Sigmoidal propagation not implemented";
			}

			int UpwardPropagator::visit(shared_ptr<Sin> sin)
			{
				double size = sin->arg->max - sin->arg->min;
				bool c = false;
				if (size <= 2 * M_PI)
				{
					double a = std::sin(sin->arg->max);
					double b = std::sin(sin->arg->min);
					double halfPI = M_PI / 2;
					double x = ceil((sin->arg->min - halfPI) / M_PI);
					double y = floor((sin->arg->max - halfPI) / M_PI);
					if (x == y)
					{ //single extrema
						if (((int)x) % 2 == 0)
						{ //maxima
							c = updateInterval(sin, std::min(a, b), 1);
						}
						else
						{ //minima
							c = updateInterval(sin, -1, std::max(a, b));
						}
					}
					else if (x > y)
					{ //no extrema
						c = updateInterval(sin, std::min(a, b), std::max(a, b));
					} //multiple extrema, don't update
				}
				if (c)
					addChanged(sin);
				return c;
			}

			int UpwardPropagator::visit(shared_ptr<Sum> sum)
			{
				double min = 0;
				double max = 0;

				for (int i = sum->terms.size() - 1; i >= 0; --i)
				{
					min += sum->terms[i]->min;
					max += sum->terms[i]->max;
				}
				if (updateInterval(sum, min, max))
				{
					addChanged(sum);
					return true;
				}
				return false;
			}

			int UpwardPropagator::visit(shared_ptr<TermPower> power)
			{
				throw "Propagation for TemPower not implemented";
			}

			int UpwardPropagator::visit(shared_ptr<Variable> var)
			{
				return true;
			}

			int UpwardPropagator::visit(shared_ptr<Zero> zero)
			{
				return false;
			}

			void UpwardPropagator::addChanged(shared_ptr<Term> t)
			{
				/*foreach(Term s in t->parents) {
				 changed->enqueue(s);
				 }
				 changed->enqueue(t);*/
				for (shared_ptr<Term> s : t->parents)
				{
					//changed->enqueue(s);
					if (!changed->contains(s))
						changed->enqueue(s);
					//changed.MoveToEnd(s);
				}
				if (!changed->contains(t))
					changed->enqueue(t);
				//changed->enqueue(t);
			}

			void UpwardPropagator::outputChange(shared_ptr<Term> t, double oldmin, double oldmax)
			{
				//Console.WriteLine("UW: Interval of {0} is now [{1}, {2}]",t,t->min,t->max);
				double oldwidth = oldmax - oldmin;
				double newwidth = t->max - t->min;
				if (dynamic_pointer_cast<Variable>(t) != 0)
					cout << "UW shrinking [" << oldmin << ".." << oldmax << "] to [" << t->min << ".." << t->max
							<< "] by " << (oldwidth - newwidth) << " (" << ((oldwidth - newwidth) / oldwidth * 100)
							<< "%)" << endl;
			}

			bool UpwardPropagator::updateInterval(shared_ptr<Term> t, double min, double max)
			{
				bool ret = t->min < min || t->max > max;
#ifdef DEBUG_UP
				double oldmin = t->min;
				double oldmax = t->max;
#endif
				if (!std::isnan(min))
					t->min = std::max(t->min, min);
				if (!std::isnan(max))
					t->max = std::min(t->max, max);
				if (ret)
					IntervalPropagator::updates++;
				IntervalPropagator::visits++;
#ifdef DEBUG_UP
				if (ret) OutputChange(t,oldmin,oldmax);
#endif
				if (t->min > t->max)
					throw new UnsolveableException();
				return ret;
			}

		} /* namespace intervalpropagation */
	} /* namespace reasoner */
} /* namespace alica */
