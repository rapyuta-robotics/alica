/*
 * DownwardPropagator.cpp
 *
 *  Created on: Dec 12, 2014
 *      Author: Philipp
 */

#include "intervals/DownwardPropagator.h"
//#define DEBUG_DP

#include "intervals/IntervalPropagator.h"
#include "intervals/UpwardPropagator.h"
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

			DownwardPropagator::DownwardPropagator()
			{
			}

			DownwardPropagator::~DownwardPropagator()
			{
				// TODO Auto-generated destructor stub
			}

			int DownwardPropagator::visit(shared_ptr<Abs> abs)
			{
				if (updateInterval(abs->arg, -abs->max, abs->max))
				{
					addChanged(abs->arg);
					return true;
				}
				return false;
			}

			int DownwardPropagator::visit(shared_ptr<And> and_)
			{
				bool changed = false;
				if (and_->min > 0)
				{
					if (updateInterval(and_->left, 1, 1))
					{
						addChanged(and_->left);
						changed = true;
					}
					if (updateInterval(and_->right, 1, 1))
					{
						addChanged(and_->right);
						changed = true;
					}
				}
				return changed;
			}

			int DownwardPropagator::visit(shared_ptr<Atan2> atan2)
			{
				throw "Atan2 propagation not implemented";
			}

			int DownwardPropagator::visit(shared_ptr<Constant> constant)
			{
				return false;
			}

			int DownwardPropagator::visit(shared_ptr<ConstPower> intPower)
			{
				if (intPower->max == numeric_limits<double>::infinity()
						|| intPower->min == -numeric_limits<double>::infinity())
					return false;
				double a = pow(intPower->min, 1 / intPower->exponent);
				double b = pow(intPower->max, 1 / intPower->exponent);

				bool isRational = intPower->exponent != round(intPower->exponent);
				if (isRational)
				{
					if (updateInterval(intPower->base, std::max(0.0, std::min(a, b)),
										std::max(a, std::max(-a, std::max(b, -b)))))
					{
						//Console.WriteLine("From DW intpower (ir) {0}",intPower);
						addChanged(intPower->base);
						return true;
					}
				}
				else
				{
					double min;
					double max;
					if (intPower->exponent >= 0)
					{
						if (intPower->base->max <= 0)
						{
							max = std::max(-abs(a), -abs(b));
						}
						else
							max = std::max(a, std::max(-a, std::max(b, -b)));
						if (intPower->base->min >= 0)
						{
							min = std::min(abs(a), abs(b));
						}
						else
							min = std::min(a, std::min(-a, std::min(b, -b)));
					}
					else
					{ //this case can be improved
						max = std::max(a, std::max(-a, std::max(b, -b)));
						min = std::min(a, std::min(-a, std::min(b, -b)));
					}
					if (updateInterval(intPower->base, min, max))
					{
						//Console.WriteLine("From DW intpower {0} [{1} : {2}]",intPower,intPower->min,intPower->max);
						addChanged(intPower->base);
						return true;
					}
				}
				return false;
			}

			int DownwardPropagator::visit(shared_ptr<ConstraintUtility> cu)
			{
				bool c = false;
				if (cu->min >= 1)
				{
					if (updateInterval(cu->constraint, 1, 1))
					{
						addChanged(cu->constraint);
						c = true;
					}
					if (updateInterval(cu->utility, 1, cu->max))
					{
						addChanged(cu->utility);
						c = true;
					}
				}
				return c;
			}

			int DownwardPropagator::visit(shared_ptr<Cos> cos)
			{
				if (cos->min == -1.0 && cos->max == 1.0)
					return false;
				double cdist = cos->arg->max - cos->arg->min;
				if (cdist >= M_PI)
					return false;
				//Console.WriteLine("Cos Prop Sine interval: [{0}, {1}]",cos->min,cos->max);
				//Console.WriteLine("Arg interval: [{0}, {1}]",cos->arg->min,cos->arg->max);
				double a = acos(cos->min);
				double b = acos(cos->max); //0..pi
				double t;
				if (a > b)
				{
					t = b;
					b = a;
					a = t;
				} //now a<= b;

				double c = -b;
				double d = -a;

				double n1 = ceil((cos->arg->min - a) / (2 * M_PI));
				//double n1a = floor((sin->arg->max - a) / (2*M_PI));
				double n2 = floor((cos->arg->max - b) / (2 * M_PI));
				//double n2a = ceil((sin->arg->min - b)   /   (2*M_PI));
				double n3 = ceil((cos->arg->min - c) / (2 * M_PI));
				//double n3a = floor((sin->arg->max - c) / (2*M_PI));
				double n4 = floor((cos->arg->max - d) / (2 * M_PI));
				//double n4a = ceil((sin->arg->min - d)   /   (2*M_PI));
				//Console.WriteLine("N: {0} {1} {2} {3}",n1,n2,n3,n4);
				//Console.WriteLine("P: {0} {1} {2} {3}",n1*2*M_PI+a,n2*2*M_PI+b,n3*2*M_PI+c,n4*2*M_PI+d);
				double min = numeric_limits<double>::max();
				double max = numeric_limits<double>::min();
				double n1a = n1 * 2 * M_PI + a;
				double n2b = n2 * 2 * M_PI + b;
				bool faulty = true;
				if (n1a <= cos->arg->max && n2b >= cos->arg->min)
				{ //interval 1 completely enclosed
					min = std::min(min, n1a);
					max = std::max(max, n2b);
					faulty = false;
				}
				else
				{ //no bound is inside as adding interval is smaller than pi
				}

				double n3c = n3 * 2 * M_PI + c;
				double n4d = n4 * 2 * M_PI + d;

				if (n3c <= cos->arg->max && n4d >= cos->arg->min)
				{ //interval 2 completely enclosed
					min = std::min(min, n3c);
					max = std::max(max, n4d);
					faulty = false;
				}
				else
				{ //no bound is inside as adding interval is smaller than pi
				}
				if (faulty)
				{
					throw new UnsolveableException();
					//return false;//return updateInterval(cos->arg,cos->arg->max,cos->arg->min); //no solution possible
				}

				if (min == numeric_limits<double>::max())
					min = numeric_limits<double>::min();
				if (max == numeric_limits<double>::min())
					max = numeric_limits<double>::max();
				if (updateInterval(cos->arg, min, max))
				{
					addChanged(cos->arg);
					return true;
				}
				return false;
			}

			int DownwardPropagator::visit(shared_ptr<Exp> exp)
			{
				double a = log(exp->min);
				double b = log(exp->max);
				if (updateInterval(exp->arg, a, b))
				{
					addChanged(exp->arg);
					return true;
				}
				return false;
			}

			int DownwardPropagator::visit(shared_ptr<Gp> gp)
			{
				throw "Propagation for TemPower not implemented";
			}

			int DownwardPropagator::visit(shared_ptr<LinSigmoid> sigmoid)
			{
				throw "Sigmoidal propagation not implemented";
			}

			int DownwardPropagator::visit(shared_ptr<Log> log)
			{
				double a = exp(log->min);
				double b = exp(log->max);
				if (updateInterval(log->arg, a, b))
				{
					addChanged(log->arg);
					return true;
				}
				return false;
			}

			int DownwardPropagator::visit(shared_ptr<LTConstraint> constraint)
			{
				bool changed = false;
				if (constraint->min > 0)
				{
					if (updateInterval(constraint->right, constraint->left->min, numeric_limits<double>::infinity()))
					{
						addChanged(constraint->right);
						changed = true;
					}
					if (updateInterval(constraint->left, -numeric_limits<double>::infinity(), constraint->right->max))
					{
						addChanged(constraint->left);
						changed = true;
					}
				}
				else if (constraint->max <= 0)
				{
					if (updateInterval(constraint->right, -numeric_limits<double>::infinity(), constraint->left->max))
					{
						addChanged(constraint->right);
						changed = true;
					}
					if (updateInterval(constraint->left, constraint->right->min, numeric_limits<double>::infinity()))
					{
						addChanged(constraint->left);
						changed = true;
					}
				}
				return changed;
			}

			int DownwardPropagator::visit(shared_ptr<LTEConstraint> constraint)
			{
				bool changed = false;
				if (constraint->min > 0)
				{
					if (updateInterval(constraint->right, constraint->left->min, numeric_limits<double>::infinity()))
					{
						addChanged(constraint->right);
						changed = true;
					}
					if (updateInterval(constraint->left, -numeric_limits<double>::infinity(), constraint->right->max))
					{
						addChanged(constraint->left);
						changed = true;
					}
				}
				else if (constraint->max <= 0)
				{
					if (updateInterval(constraint->right, -numeric_limits<double>::infinity(), constraint->left->max))
					{
						addChanged(constraint->right);
						changed = true;
					}
					if (updateInterval(constraint->left, constraint->right->min, numeric_limits<double>::infinity()))
					{
						addChanged(constraint->left);
						changed = true;
					}
				}
				return changed;
			}

			int DownwardPropagator::visit(shared_ptr<Max> max)
			{
				if (max->min > 0)
				{
					bool c = false;
					if (max->left->max <= 0)
					{
						bool c1 = updateInterval(max->right, 1, 1);
						if (c1)
							addChanged(max->right);
						c |= c1;
					}
					if (max->right->max <= 0)
					{
						bool c2 = updateInterval(max->left, 1, 1);
						if (c2)
							addChanged(max->left);
						c |= c2;
					}
					return c;
				}
				//bool c3 = updateInterval(max->left,numeric_limits<double>::min(),max->max);
				//bool c4 = updateInterval(max->right,numeric_limits<double>::min(),max->max);
				return false;
			}

			int DownwardPropagator::visit(shared_ptr<Min> min)
			{
				bool c1 = updateInterval(min->left, min->min, numeric_limits<double>::infinity());
				bool c2 = updateInterval(min->right, min->min, numeric_limits<double>::infinity());
				if (c1)
					addChanged(min->left);
				if (c2)
					addChanged(min->right);
				return c1 || c2;
			}

			int DownwardPropagator::visit(shared_ptr<Or> or_)
			{
				throw "Or operator progation not implemented (max is used)";
			}

			int DownwardPropagator::visit(shared_ptr<Product> product)
			{
				/*
				 * a*b = c
				 * ==> a = c/b
				 * */
				if (product->left == product->right)
				{
					double a = sqrt(product->min);
					double b = sqrt(product->max);
					double min;
					double max;
					if (product->left->max <= 0)
					{
						max = std::max(-a, -b);
					}
					else
						max = std::max(a, b);
					if (product->left->min >= 0)
					{
						min = std::min(a, b);
					}
					else
						min = std::min(-a, -b);
					if (updateInterval(product->left, min, max))
					{
						addChanged(product->left);
						return true;
					}

				}
				else
				{
					bool c = false, d = false;
					if (product->right->min * product->right->max > 0)
					{
						//Left:
						double aa = product->min / product->right->min;
						double ab = product->min / product->right->max;
						double ba = product->max / product->right->min;
						double bb = product->max / product->right->max;

						double min = std::min(aa, std::min(ab, std::min(ba, bb)));
						double max = std::max(aa, std::max(ab, std::max(ba, bb)));

						c = updateInterval(product->left, min, max);
						if (c)
							addChanged(product->left);
					}
					if (product->left->min * product->left->max > 0)
					{
						//Right:
						double aa = product->min / product->left->min;
						double ab = product->min / product->left->max;
						double ba = product->max / product->left->min;
						double bb = product->max / product->left->max;

						double min = std::min(aa, std::min(ab, std::min(ba, bb)));
						double max = std::max(aa, std::max(ab, std::max(ba, bb)));

						d = updateInterval(product->right, min, max);
						if (d)
							addChanged(product->right);

					}
					return c || d;
				}
				return false;
			}

			int DownwardPropagator::visit(shared_ptr<Reification> reif)
			{
				bool c = false;
				if (reif->max < reif->max)
				{
					c = updateInterval(reif, reif->min, reif->min);
					if (c)
						addChanged(reif);
					if (updateInterval(reif->condition, -numeric_limits<double>::infinity(), 0))
					{
						addChanged(reif->condition);
						c = true;
					}
				}
				else if (reif->min > reif->min)
				{
					c = updateInterval(reif, reif->max, reif->max);
					if (c)
						addChanged(reif);
					if (updateInterval(reif->condition, 1, 1))
					{
						addChanged(reif->condition);
						c = true;
					}
				}

				return c;
			}

			int DownwardPropagator::visit(shared_ptr<Sigmoid> sigmoid)
			{
				throw "Sigmoidal propagation not implemented";
			}

			int DownwardPropagator::visit(shared_ptr<Sin> sin)
			{
				if (sin->min == -1.0 && sin->max == 1.0)
					return false;
				double cdist = sin->arg->max - sin->arg->min;
				if (cdist >= M_PI)
					return false;
				//Console.WriteLine("Sine Prop Sine interval: [{0}, {1}]",sin->min,sin->max);
				//Console.WriteLine("Arg interval: [{0}, {1}]",sin->arg->min,sin->arg->max);
				double a = asin(sin->min);
				double b = asin(sin->max); //-pi/2..pi/2
				double t;
				if (a > b)
				{
					t = b;
					b = a;
					a = t;
				} //now a<= b;

				double c = M_PI - b;
				double d = M_PI - a;

				double n1 = ceil((sin->arg->min - a) / (2 * M_PI));
				//double n1a = floor((sin->arg->max - a) / (2*M_PI));
				double n2 = floor((sin->arg->max - b) / (2 * M_PI));
				//double n2a = ceil((sin->arg->min - b)   /   (2*M_PI));
				double n3 = ceil((sin->arg->min - c) / (2 * M_PI));
				//double n3a = floor((sin->arg->max - c) / (2*M_PI));
				double n4 = floor((sin->arg->max - d) / (2 * M_PI));
				//double n4a = ceil((sin->arg->min - d)   /   (2*M_PI));
				//Console.WriteLine("N: {0} {1} {2} {3}",n1,n2,n3,n4);
				//Console.WriteLine("P: {0} {1} {2} {3}",n1*2*M_PI+a,n2*2*M_PI+b,n3*2*M_PI+c,n4*2*M_PI+d);
				double min = numeric_limits<double>::max();
				double max = numeric_limits<double>::min();
				double n1a = n1 * 2 * M_PI + a;
				double n2b = n2 * 2 * M_PI + b;
				bool faulty = true;
				if (n1a <= sin->arg->max && n2b >= sin->arg->min)
				{ //interval 1 completely enclosed
					min = std::min(min, n1a);
					max = std::max(max, n2b);
					faulty = false;
				}
				else
				{ //no bound is inside as adding interval is smaller than pi
				}

				double n3c = n3 * 2 * M_PI + c;
				double n4d = n4 * 2 * M_PI + d;

				if (n3c <= sin->arg->max && n4d >= sin->arg->min)
				{ //interval 2 completely enclosed
					min = std::min(min, n3c);
					max = std::max(max, n4d);
					faulty = false;
				}
				else
				{ //no bound is inside as adding interval is smaller than pi
				}

				if (faulty)
				{
					throw new UnsolveableException();
					//return false; //updateInterval(sin->arg,sin->arg->max,sin->arg->min); //no solution possible
				}
				/*if (n1 == n2) { //bound within interval
				 min = std::min(min,n1*2*M_PI+a);
				 max = std::max(max,n2*2*M_PI+b);
				 } else {
				 if (n1 > n2) { //lower bound cut

				 min = std::min(min,sin->arg->min);
				 max = std::max(max,n2*2*M_PI+b);

				 double k =

				 }
				 }

				 //if (n1 == n2 && n3 == n4) { //bind to rectangle:
				 double min = std::min(n1*2*M_PI+a,n3*2*M_PI+c);
				 double max = std::max(n2*2*M_PI+b,n4*2*M_PI+d);
				 */
				//}
				if (min == numeric_limits<double>::max())
					min = -numeric_limits<double>::infinity();
				if (max == numeric_limits<double>::min())
					max = numeric_limits<double>::infinity();
				if (updateInterval(sin->arg, min, max))
				{
					addChanged(sin->arg);
					return true;
				}
				return false;
			}

			int DownwardPropagator::visit(shared_ptr<Sum> sum)
			{
				//a+b= c
				// a= b-c
				//a:
				bool changed = false;
				bool anychange = false;
				do
				{
					changed = false;
					for (int i = 0; i < sum->terms.size(); ++i)
					{
						double minother = 0;
						double maxother = 0;
						for (int j = 0; j < sum->terms.size(); ++j)
						{
							if (i == j)
								continue;
							minother += sum->terms[j]->min;
							maxother += sum->terms[j]->max;
						}
						/*Console.WriteLine("-______S({0} {1})",sum->min,sum->max);
						 Console.WriteLine("-______O({0} {1})",maxother,minother);
						 Console.WriteLine("-______>DW {0} to {1} {2} I am {3}",t,sum->min-maxother,sum->max-minother,sum);
						 */
						if (updateInterval(sum->terms[i], sum->min - maxother, sum->max - minother))
						{
							addChanged(sum->terms[i]);
							changed = true;
							anychange = true;
						}
					}
				} while (changed);
				return anychange;
			}

			int DownwardPropagator::visit(shared_ptr<TermPower> power)
			{
				throw "Propagation for TemPower not implemented";
			}

			int DownwardPropagator::visit(shared_ptr<Variable> var)
			{
				return false;
			}

			int DownwardPropagator::visit(shared_ptr<Zero> zero)
			{
				return false;
			}

			void DownwardPropagator::addChanged(shared_ptr<Term> t)
			{
				if (!changed->contains(t))
				{
					changed->enqueue(t);
				}
			}

			void DownwardPropagator::outputChange(shared_ptr<Term> t, double oldmin, double oldmax)
			{
				double oldwidth = oldmax - oldmin;
				double newwidth = t->max - t->min;
				if (dynamic_pointer_cast<Variable>(t) != 0)
					cout << "DW shrinking [" << oldmin << ".." << oldmax << "] to [" << t->min << ".." << t->max
							<< "] by " << (oldwidth - newwidth) << " (" << ((oldwidth - newwidth) / oldwidth * 100)
							<< "%)" << endl;
			}

			bool DownwardPropagator::updateInterval(shared_ptr<Term> t, double min, double max)
			{
				bool ret = t->min < min || t->max > max;
#ifdef DEBUG_DP
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
#ifdef DEBUG_DP
				if (ret) OutputChange(t,oldmin,oldmax);
#endif
				if (t->min > t->max)
					throw new UnsolveableException();
				return ret;
			}
		} /* namespace intervalpropagation */
	} /* namespace reasoner */
} /* namespace alica */
