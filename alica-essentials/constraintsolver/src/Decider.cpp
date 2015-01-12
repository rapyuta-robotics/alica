/*
 * Decider.cpp
 *
 *  Created on: Dec 11, 2014
 *      Author: Philipp
 */

#include "Decider.h"

#include "types/Clause.h"
#include "types/DecisionLevel.h"
#include "types/Lit.h"
#include "types/Var.h"
#include "types/Watcher.h"
#include "CNSat.h"

#include <limits>
#include <algorithm>

#include <iostream>

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{

			shared_ptr<Var> Decider::decideRangeBased(shared_ptr<vector<shared_ptr<Var> > > variables,
														shared_ptr<CNSat> solver)
			{
				vector<shared_ptr<Lit>> choices;
				int vars = variables->size();
				for (int i = 0; i < vars; i++)
				{
					if (variables->at(i)->assignment != Assignment::UNASSIGNED)
						continue;
					bool hT = false;
					bool hF = false;
					for (int j = 0; j < variables->at(i)->watchList->size(); j++)
					{
						if (!variables->at(i)->watchList->at(j)->clause->satisfied)
						{
							if (variables->at(i)->watchList->at(j)->lit->sign == Assignment::TRUE)
							{
								if (!hT)
									choices.push_back(variables->at(i)->watchList->at(j)->lit);
								hT = true;

							}
							else
							{
								if (!hF)
									choices.push_back(variables->at(i)->watchList->at(j)->lit);
								hF = true;
							}
						}
						if (hT && hF)
							break;
					}
				}
				if (choices.size() == 0)
					return nullptr;

				stable_sort(choices.begin(), choices.end(), litRangeCompare);
				/*Console.WriteLine("======");
				 foreach(Lit l in choices) {
				 Console.WriteLine("{0}",(l->sign == Assignment::TRUE ? l->var->positiveRangeSize:l->var->negativeRangeSize));
				 }
				 Console.WriteLine("======");*/
				//Choices are now in ascending order!
				shared_ptr<Var> v;
				Assignment ass;

				//Uniform:
				//int idx = solver.Rand.Next(choices.size());
				//smallest range:
				//int idx = 0;
				//largest range:
				int idx = choices.size() - 1;
				//Prefer smaller:
				/*int idx = choices.size()-1;
				 double r = solver.Rand.NextDouble();
				 double q = 0;
				 for(int i=0; i<choices.size(); i++) {
				 q += 1.0/(i+2);
				 if (q > r) {
				 idx = i;
				 break;
				 }
				 }*/
				//Prefer larger:
				/*int idx = 0;
				 double r = solver.Rand.NextDouble();
				 double q = 0;
				 for(int i=0; i<choices.size(); i++) {
				 q += 1.0/(i+2);
				 if (q > r) {
				 idx = choices.size()-i-1;
				 break;
				 }
				 }*/
				//Pick middle:
				//int idx = choices.size()/2;
				v = choices.at(idx)->var;
				ass = choices.at(idx)->sign;

				shared_ptr<DecisionLevel> d = make_shared<DecisionLevel>(solver->decisions->size());
				solver->decisionLevel->push_back(d);
				v->assignment = ass;
				solver->decisions->push_back(v);
				v->setReason(nullptr);
				v->decisionLevel = d;

				return v;
			}

			shared_ptr<Var> Decider::decideActivityBased(shared_ptr<vector<shared_ptr<Var> > > variables,
															shared_ptr<CNSat> solver)
			{
				shared_ptr<CNSMTGSolver> cnsmtGSolver = nullptr;
				if (solver->cnsmtGSolver.use_count() > 0)
				{
					cnsmtGSolver = solver->cnsmtGSolver.lock();
				}

				int vars = variables->size();
				int init = ((double)rand() / RAND_MAX) * vars;
				shared_ptr<Var> v = nullptr, next = nullptr;
				int maxActivity = 0;
				//Search Lit with highest Activity
				if (cnsmtGSolver == nullptr)
				{
					for (int i = 0; i < vars; i++)
					{
						int p = (init + i) % vars;
						if (p < 0 || p >= vars)
							cout << "p = " << p << endl;
						v = variables->at(p);
						if (v->assignment == Assignment::UNASSIGNED)
						{
							if (maxActivity <= v->activity)
							{
								maxActivity = v->activity;
								next = v;
							}
						}
					}
					//Decide it
					if (next != nullptr)
					{
						shared_ptr<DecisionLevel> d = make_shared<DecisionLevel>(solver->decisions->size());
						solver->decisionLevel->push_back(d);

						double rel = next->positiveAppearance + next->negativeAppearance;
						if (rel != 0)
							rel = ((double)next->positiveAppearance) / rel;
						else
							rel = 0.5;

						next->assignment = ((double)rand() / RAND_MAX < rel) ? Assignment::TRUE : Assignment::FALSE;
						solver->decisions->push_back(next);
						next->setReason(nullptr);
						next->decisionLevel = d;
						return next;
					}
				}
				else
				{
					init = ((double)rand() / RAND_MAX) * solver->clauses->size();
					for (int i = 0; i < solver->clauses->size(); i++)
					{
						shared_ptr<Clause> c = solver->clauses->at((i + init) % solver->clauses->size());
						if (!c->satisfied && c->literals->size() > 1)
						{
							if (!c->watcher->at(0)->lit->satisfied())
							{
								next = c->watcher->at(0)->lit->var;
								shared_ptr<DecisionLevel> d = make_shared<DecisionLevel>(solver->decisions->size());
								solver->decisionLevel->push_back(d);

								next->assignment = c->watcher->at(0)->lit->sign;
								solver->decisions->push_back(next);
								next->setReason(nullptr);
								next->decisionLevel = d;
								return next;
							}
							else if (!c->watcher->at(1)->lit->satisfied())
							{
								shared_ptr<DecisionLevel> d = make_shared<DecisionLevel>(solver->decisions->size());
								solver->decisionLevel->push_back(d);
								next = c->watcher->at(1)->lit->var;

								next->assignment = c->watcher->at(1)->lit->sign;
								solver->decisions->push_back(next);
								next->setReason(nullptr);
								next->decisionLevel = d;
								return next;
							}
							else
								cout << "This shoud Never Happen!!" << endl;
						}
					}
				}

				return nullptr;
			}

			shared_ptr<Var> Decider::decideVariableCountBased(shared_ptr<vector<shared_ptr<Var> > > variables,
																shared_ptr<CNSat> solver)
			{
				int vars = variables->size();
				shared_ptr<Lit> l = nullptr;
				int maxCount = numeric_limits<int>::max();
				int minCount = -1;

				for (int i = 0; i < vars; i++)
				{
					if (variables->at(i)->assignment != Assignment::UNASSIGNED)
						continue;
					for (int j = 0; j < variables->at(i)->watchList->size(); j++)
					{
						if (!variables->at(i)->watchList->at(j)->clause->satisfied)
						{
							/*if(maxCount>variables->at(i)->watchList->at(j)->lit->variableCount) {
							 l = variables->at(i)->watchList->at(j)->lit;
							 maxCount = l->variableCount;
							 }*/
							if (minCount < variables->at(i)->watchList->at(j)->lit->variableCount)
							{
								l = variables->at(i)->watchList->at(j)->lit;
								minCount = l->variableCount;
							}
						}
					}
				}
				if (l == nullptr)
					return nullptr;
				Assignment ass;
				shared_ptr<Var> v = l->var;

				ass = l->sign;

				shared_ptr<DecisionLevel> d = make_shared<DecisionLevel>(solver->decisions->size());
				solver->decisionLevel->push_back(d);
				v->assignment = ass;
				solver->decisions->push_back(v);
				v->setReason(nullptr);
				v->decisionLevel = d;

				return v;
			}

			bool Decider::litRangeCompare(shared_ptr<Lit> a, shared_ptr<Lit> b)
			{
				double sa;
				double sb;
				if (a->sign == Assignment::TRUE)
					sa = a->var->positiveRangeSize;
				else
					sa = a->var->negativeRangeSize;
				if (b->sign == Assignment::TRUE)
					sb = b->var->positiveRangeSize;
				else
					sb = b->var->negativeRangeSize;
				return sa > sb;
			}
		} /* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */
