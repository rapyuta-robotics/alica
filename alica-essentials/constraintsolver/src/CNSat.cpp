/*
 * CNSat.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#include "CNSat.h"
//#define CNSatDebug

#include "types/Clause.h"
#include "types/DecisionLevel.h"
#include "types/Lit.h"
#include "types/Var.h"
#include "types/Watcher.h"
#include "CNSMTGSolver.h"
#include "Decider.h"

#include <engine/IAlicaClock.h>
#include <clock/AlicaROSClock.h>

#include <algorithm>

#include <iostream>

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{

			CNSat::CNSat()
			{
				this->useIntervalProp = true;
				this->decisionLevelNull = make_shared<DecisionLevel>(0);
				this->unitDecissions = 0;
				this->decisions = make_shared<vector<shared_ptr<Var> >>();
				this->decisionLevel = make_shared<vector<shared_ptr<DecisionLevel> >>();
				this->clauses = make_shared<vector<shared_ptr<Clause> >>();
				this->variables = make_shared<vector<shared_ptr<Var> >>();

				this->clauses = make_shared<vector<shared_ptr<Clause> >>();
				this->satClauses = make_shared<vector<shared_ptr<Clause> >>();
				this->tClauses = make_shared<vector<shared_ptr<Clause> >>();
				this->iClauses = make_shared<vector<shared_ptr<Clause> >>();
				this->variables = make_shared<vector<shared_ptr<Var> >>();
				this->decisions = make_shared<vector<shared_ptr<Var> >>();
				this->decisionLevel = make_shared<vector<shared_ptr<DecisionLevel> >>();

				alicaClock = new alicaRosProxy::AlicaROSClock();
			}

			CNSat::~CNSat()
			{
				// TODO Auto-generated destructor stub
			}

			shared_ptr<Var> CNSat::newVar()
			{
				shared_ptr<Var> t = make_shared<Var>(variables->size());
				variables->push_back(t);
				return t;
			}

			bool CNSat::addBasicClause(shared_ptr<Clause> c)
			{
				if (c->literals->size() == 0)
					return false;

				if (c->literals->size() > 1)
				{
					shared_ptr<Watcher> w1 = make_shared<Watcher>(c->literals->at(0), c);
					shared_ptr<Watcher> w2 = make_shared<Watcher>(c->literals->at(1), c);
					c->watcher->at(0) = w1;
					c->watcher->at(1) = w2;
					clauses->push_back(c);
					/*for(shared_ptr<Lit> l in c->literals) {
					 if(l->sign == Assignment::TRUE) l.var->activity++;
					 else l->var.NegActivity++;
					 }*/
				}
				else
				{
					if (c->literals->at(0)->var->assignment != Assignment::UNASSIGNED
							&& c->literals->at(0)->var->assignment != c->literals->at(0)->sign)
						return false;

					c->literals->at(0)->var->assignment = c->literals->at(0)->sign;
					c->literals->at(0)->var->decisionLevel = this->decisionLevelNull;
					c->literals->at(0)->var->setClause(nullptr);

					//TODO check this!!!
					decisions->push_back(c->literals->at(0)->var);
					c->literals->at(0)->var->locked = true;
					clauses->push_back(c);
				}

				return true;
			}

			void CNSat::emptySATClause()
			{
				emptyClauseList(this->satClauses);
			}

			void CNSat::emptyTClause()
			{
				emptyClauseList(this->tClauses);
			}

			void CNSat::resetVariables()
			{
				for (shared_ptr<Var> v : *variables)
				{
					v->reset();
				}
			}

			bool CNSat::addSATClause(shared_ptr<Clause> c)
			{
				if (c->literals->size() == 1)
				{
					//TODO Can be removed when Locked is removed
					if (c->literals->at(0)->var->locked
							&& c->literals->at(0)->sign != c->literals->at(0)->var->assignment)
					{
						return false;
					}

					///////////
					///////////
					/////////// AAAAAAAAAAAAAAAAm Ende von DL 0 muss das sein! oO
					///////////
					///////////
					/*if(decisionLevelNull->level-1 > decisions->size()) decisions->push_back(c->literals->at(0)->var);
					 else
					 decisions->insert(decisions->begin()+decisionLevelNull->level-1, c->literals->at(0)->var);				*/
					/*if(DecisionLevel->size()>1)decisions.Insert(DecisionLevel->at(1)->level-1, c->literals->at(0)->var);
					 else
					 decisions->push_back(c->literals->at(0)->var);*/
					decisions->insert(decisions->begin() + decisionLevelNull->level, c->literals->at(0)->var);
					//decisions.Insert(0, c->literals->at(0)->var);
					c->literals->at(0)->var->decisionLevel = this->decisionLevel->at(0);
					c->literals->at(0)->var->assignment = c->literals->at(0)->sign;
					c->literals->at(0)->var->setClause(nullptr);
					c->literals->at(0)->var->locked = true;
					for (shared_ptr<DecisionLevel> l : *(this->decisionLevel))
					{
						l->level++;
					}
					return true;
				}
				shared_ptr<Watcher> w1 = make_shared<Watcher>(c->literals->at(0), c);
				shared_ptr<Watcher> w2 = make_shared<Watcher>(c->literals->at(c->literals->size() - 1), c);
				c->watcher->at(0) = w1;
				c->watcher->at(1) = w2;
				satClauses->push_back(c);

				return true;
			}

			bool CNSat::addTClause(shared_ptr<Clause> c)
			{
				if (c->literals->size() == 1)
				{
					if (c->literals->at(0)->var->locked
							&& c->literals->at(0)->sign != c->literals->at(0)->var->assignment)
						return false;

					//TODO Check: Do we need this?
					backTrack(this->decisionLevel->at(0));

					decisions->insert(decisions->begin() + decisionLevelNull->level, c->literals->at(0)->var);
					//decisions.Insert(0, c->literals->at(0)->var);
					c->literals->at(0)->var->decisionLevel = this->decisionLevel->at(0);
					c->literals->at(0)->var->assignment = c->literals->at(0)->sign;
					c->literals->at(0)->var->setClause(nullptr);
					//Do we have to Lock T-Clauses?????
					//c->literals->at(0)->var->locked = true;
					for (shared_ptr<DecisionLevel> l : *(this->decisionLevel))
					{
						l->level++;
					}
					return true;
				}
				shared_ptr<Watcher> w1 = make_shared<Watcher>(c->literals->at(c->literals->size() - 2), c);
				shared_ptr<Watcher> w2 = make_shared<Watcher>(c->literals->at(c->literals->size() - 1), c);
				c->watcher->at(0) = w1;
				c->watcher->at(1) = w2;
				tClauses->push_back(c);

				return true;
			}

			bool CNSat::preAddIUnitClause(shared_ptr<Var> v, Assignment ass)
			{
				if ((v->assignment != Assignment::UNASSIGNED) && (v->assignment != ass))
				{
					return false; //problem is unsolveable
				}
				decisions->insert(decisions->begin(), v);
				v->decisionLevel = this->decisionLevel->at(0);
				v->assignment = ass;
				v->setClause(nullptr);
				v->locked = true;
				this->decisionLevel->at(0)->level++;
				unitDecissions++;
				return true;
			}

			bool CNSat::addIClause(shared_ptr<Clause> c)
			{
				if (c->literals->size() == 1)
				{
					//TODO brauchen wir hier noch einen check, ob da schon was gesetzt ist?
					//if (c->literals->at(0)->var->locked && c->literals->at(0)->sign != c->literals->at(0)->var->assignment)
					//	return false;
					//backTrackAndRevoke(c->literals->at(0)->var->decisionLevel);
					backTrack(this->decisionLevel->at(0));

					decisions->insert(decisions->begin(), c->literals->at(0)->var);
					c->literals->at(0)->var->decisionLevel = this->decisionLevel->at(0);
					c->literals->at(0)->var->assignment = c->literals->at(0)->sign;
					c->literals->at(0)->var->setClause(nullptr);
					c->literals->at(0)->var->locked = true;
					for (shared_ptr<DecisionLevel> l : *(this->decisionLevel))
					{
						l->level++;
					}
					unitDecissions++;
					return true;
				}
				//TODO this does somehow not work! oO
				//shared_ptr<Watcher> w1 = make_shared<Watcher>(c->literals[c->literals->size()-2], c);
				shared_ptr<Watcher> w1 = make_shared<Watcher>(c->literals->at(0), c);
				shared_ptr<Watcher> w2 = make_shared<Watcher>(c->literals->at(c->literals->size() - 1), c);
				c->watcher->at(0) = w1;
				c->watcher->at(1) = w2;
				iClauses->push_back(c);

				return true;
			}

			void CNSat::init()
			{
				this->unitDecissions = decisions->size();
				this->decisionLevel->clear();
				this->decisionLevelNull->level = decisions->size();
				this->decisionLevel->push_back(this->decisionLevelNull);

				for (shared_ptr<Clause> cl : *clauses)
				{
					if (cl->literals->size() == 1)
					{
						cl->satisfied = true;
					}
				}
				this->recentBacktrack = false;
			}

			bool CNSat::solve()
			{
				shared_ptr<CNSMTGSolver> cnsmtGSolver = nullptr;
				if (this->cnsmtGSolver.use_count() > 0)
				{
					cnsmtGSolver = this->cnsmtGSolver.lock();
				}

				int restartNum = 100;
				learntNum = 700;
				restartCount = 0;
				shared_ptr<vector<shared_ptr<vector<double>>> > curRanges = nullptr;
				shared_ptr<vector<double>> solution = nullptr;
				shared_ptr<DecisionLevel> evaluatedDL = nullptr;

				shared_ptr<Clause> c;

				while (true)
				{
					c = nullptr;
					while ((c = propagate()) != nullptr) //resolve all conflicts
					{
						if (decisionLevel->size() == 1)
						{
							return false;
						}
						if (conflictCount % 50 == 0 && cnsmtGSolver != nullptr
								&& cnsmtGSolver->begin + cnsmtGSolver->maxSolveTime < alicaClock->now())
						{
							return false;
						}
						if (!resolveConflict(c))
						{
							return false;
						}
					}

					if (cnsmtGSolver != nullptr)
					{
						//check for conflict of Theoremprover
						if (useIntervalProp && !cnsmtGSolver->intervalPropagate(decisions, curRanges))
						{
							continue;
						}
						else
						{
							//TODO: Heuristic Decision whether or not to query the T-solver
							//comes in here
							//double satRatio = ((double)satClauseCount) / clauses->size();
							//double varRatio = ((double)decisions->size()) / variables->size();
							//if (recentBacktrack || varRatio < r.NextDouble()) { //|| satRatio > r.NextDouble()) {
							//if (recentBacktrack || satRatio > r.NextDouble()) {
							//if (decisionCount % 10 == 0) {
							//	recentBacktrack = false;
							//if(solution==null || !SolutionInsideRange(solution, curRanges)) {
							//if(!VarAssignmentInsideRange(Decisions[decisions->size()-1],curRanges)) {
							//if(evaluatedDL==null || !AssignmentInsideRange(evaluatedDL, curRanges)) {
//								if (!CNSMTGSolver.ProbeForSolution(decisions, out solution)) {
//									continue;
//								}
							//	evaluatedDL = DecisionLevel[DecisionLevel->size()-1];
							//}
							//}

							if (!cnsmtGSolver->probeForSolution(decisions, solution))
							{
								continue;
							}
							int satClauseCount = 0;
							for (int i = clauses->size() - 1; i >= 0; --i)
							{
								if (clauses->at(i)->satisfied)
								{
									satClauseCount++;
								}
							}
							if (satClauseCount >= clauses->size())
							{
								return true;
							}
						}
					}
					shared_ptr<Var> next;
					if (cnsmtGSolver != nullptr)
					{
						next = Decider::decideVariableCountBased(variables, shared_from_this());
					}
					else
					{
						next = Decider::decideActivityBased(variables, shared_from_this());
					}

					if (next == nullptr)
					{ // if no unassigned vars
						cout << "ConflictCount: " << conflictCount << " DecisionCount " << decisionCount << " LC "
								<< this->learnedCount << endl;
						return true;
					}
#ifdef CNSatDebug
					cout << "Decision: ";
					next->print();
					cout << endl;
#endif
					++decisionCount;
					if (decisionCount % 25 == 0 && cnsmtGSolver != nullptr
							&& cnsmtGSolver->begin + cnsmtGSolver->maxSolveTime < alicaClock->now())
						return false;
					//Forget unused clauses
					if (decisionCount % 1000 == 0)
					{
						reduceDB(learntNum);
						for (shared_ptr<Var> v : *variables)
						{
							v->activity /= 4;
						}
					}

					if (false && decisionCount % restartNum == 0)
					{
						//perform restart
						restartNum *= 2;
						learntNum += learntNum / 10;
						restartCount++;
						for (int j = (decisionLevel->at(1)->level); j < decisions->size(); j++)
						{
							decisions->at(j)->assignment = Assignment::UNASSIGNED;
							decisions->at(j)->setClause(nullptr);
							for (shared_ptr<Watcher> wa : *(decisions->at(j)->watchList))
							{
								wa->clause->satisfied = false;
							}
						}
						decisions->erase(decisions->begin() + decisionLevel->at(1)->level,
											decisions->begin() + (decisions->size() - decisionLevel->at(1)->level));

						decisionLevel->erase(decisionLevel->begin() + 1,
												decisionLevel->begin() + (decisionLevel->size() - 1));
					}
				}
			}

			void CNSat::reduceDB(int num)
			{
				if (satClauses->size() < num)
					return;
				stable_sort(satClauses->begin(), satClauses->end(), Clause::compareTo);
				for (int i = num; i < satClauses->size(); i++)
				{
					{
						auto iter = find(satClauses->at(i)->watcher->at(0)->lit->var->watchList->begin(),
											satClauses->at(i)->watcher->at(0)->lit->var->watchList->end(),
											satClauses->at(i)->watcher->at(0));
						satClauses->at(i)->watcher->at(0)->lit->var->watchList->erase(iter);
					}
					{
						auto iter = find(satClauses->at(i)->watcher->at(1)->lit->var->watchList->begin(),
											satClauses->at(i)->watcher->at(1)->lit->var->watchList->end(),
											satClauses->at(i)->watcher->at(1));
						satClauses->at(i)->watcher->at(1)->lit->var->watchList->erase(iter);
					}
					//shared_ptr<Lit> l = satClauses->at(i)->literals[satClauses->at(i)->literals->size()-1];
					//if (l->var->getClause() == satClauses->at(i)) l->var->setClause(nullptr);
				}

				satClauses->erase(satClauses->begin() + num, satClauses->begin() + (satClauses->size() - num));
				for (int i = 0; i < satClauses->size(); i++)
				{
					satClauses->at(i)->activity /= 4;
				}
			}

			shared_ptr<Clause> CNSat::propagate()
			{
				int lLevel = 0;
				if (decisionLevel->size() > 1)
				{
					lLevel = decisionLevel->at(decisionLevel->size() - 1)->level;
				}

				for (int i = lLevel; i < decisions->size(); ++i)
				{
					shared_ptr<vector<shared_ptr<Watcher>>> watchList = decisions->at(i)->watchList;

					for (int j = 0; j < watchList->size(); ++j)
					{
						shared_ptr<Watcher> w = watchList->at(j);
#ifdef CNSatDebug
						decisions->at(i)->print();
						cout << " -> " << endl;
						w->clause->print();
#endif
						if (w->clause->satisfied)
						{
							continue;
						}
						if (w->lit->satisfied())
						{
							w->clause->satisfied = true;
							continue;
						}
						//TODO Do we need this?
						if (w->lit->var->assignment == Assignment::UNASSIGNED)
						{
							continue;
						}

						//This can be optimized !?
						shared_ptr<Clause> c = w->clause;

						//Search for new Watch
						int oWId = c->watcher->at(0) == w ? 1 : 0;
						if (c->watcher->at(oWId)->lit->satisfied())
						{
							//TODO: Do we need this?
							w->clause->satisfied = true;
							continue;
						}
						bool found = false;
						for (shared_ptr<Lit> l : *(c->literals))
						{
							if (c->watcher->at(oWId)->lit->var != l->var && (l->var->assignment == Assignment::UNASSIGNED || l->satisfied()))
							{
								auto iter = find(w->lit->var->watchList->begin(), w->lit->var->watchList->end(), w);
								w->lit->var->watchList->erase(iter);
								j--;
								w->lit = l;
								l->var->watchList->push_back(w);
								found = true;
								if (l->satisfied())
								{
									w->clause->satisfied = true;
								}
								break;
							}
						}
						if (!found)
						{
							c->activity++;
							//TODO Handle shared_ptr<Watcher> here ... do not return -> faster
							shared_ptr<Watcher> w2 = c->watcher->at(oWId);
							if (w2->lit->var->assignment == Assignment::UNASSIGNED)
							{
								w2->lit->var->assignment = w2->lit->sign;
								w2->clause->satisfied = true;
								w2->lit->var->decisionLevel = decisionLevel->at(decisionLevel->size()-1);
								decisions->push_back(w2->lit->var);
								w2->lit->var->setClause(c);

								for (shared_ptr<Watcher> wi : *(w2->lit->var->watchList))
								{
									wi->clause->lastModVar = w2->lit->var;
								}
							}
							else
							{
								return c;
							}
						}
					}
				}
				return nullptr;
			}

			bool CNSat::resolveConflict(shared_ptr<Clause> c)
			{
				++conflictCount;

				//Learn shared_ptr<Clause> from conflict here
				shared_ptr<Clause> confl = c;
				shared_ptr<Clause> learnt = make_shared<Clause>();
				int index = decisions->size() - 1;
				int pathC = 0;
				shared_ptr<Var> p = nullptr;
#ifdef CNSatDebug
				cout << "\nxxxxxxxxxxxxxxxxxxx" << endl;
				cout << "\nAlready Learned" << endl;
				for (shared_ptr<Clause> a : *satClauses)
				{
					a->print();
				}

				cout << "\nAssignment" << endl;
				this->printAssignments();

				cout << "\nConflict" << endl;
				confl->print();

				cout << "\nReason" << endl;
				if (confl->lastModVar != nullptr && confl->lastModVar->getClause() != nullptr)
				confl->lastModVar->getClause()->print();
				else
				cout << "null" << endl;

				cout << "-------------------\nLearning" << endl;
#endif
				//Find all Literals until First Unique Implication Point(UIP)
				do
				{
#ifdef CNSatDebug
					if (p != nullptr)
					{
						cout << "shared_ptr<Var> ";
						p->print();
						cout << " -> ";
					}
					confl->print();
					cout << "Trying ";
#endif

					shared_ptr<Clause> cl = confl;
					//Inspect conflict reason clause Literals
					for (int j = 0; j < cl->literals->size(); j++)
					{
						shared_ptr<Lit> q = cl->literals->at(j);
						//ignore UIP
						if (q->var == p)
						{
#ifdef CNSatDebug
							q->var->print();
							cout << " n(sub) ";
#endif
							continue;
						}
						//ignore sawnvariables and decissionlevel 0
						if (!q->var->seen && q->var->decisionLevel != decisionLevel->at(0))
						{
							q->var->seen = true;
							//if q has been decided in curent level: increase iterations; else add literal to learnt clause
							if (q->var->decisionLevel->level >= (decisionLevel->at(decisionLevel->size() - 1)->level))
							{
								pathC++;
#ifdef CNSatDebug
								q->var->print();
								cout << " n(curlvl) ";
#endif
							}
							else
							{
#ifdef CNSatDebug
								q->var->print();
								cout << " add ";
#endif
								learnt->add(q);
							}
						}
					}

					// Select next clause to look at:
					//do { if(index<0) {cout << "BLA" << endl; return false;} }
					while (!decisions->at(index--)->seen)
						;

					p = decisions->at(index + 1);
					confl = p->getClause();
					p->seen = false;
					pathC--;
#ifdef CNSatDebug
					cout << endl;
#endif
				} while (pathC > 0);
#ifdef CNSatDebug
				cout << "-------------------" << endl;
#endif
				//Add UIP
				shared_ptr<Lit> t = make_shared<Lit>(
						p, (p->assignment == Assignment::FALSE) ? Assignment::TRUE : Assignment::FALSE);
				learnt->add(t);

				//Store Seen Variables for later reset
				shared_ptr<vector<shared_ptr<Lit>>> seenList = make_shared<vector<shared_ptr<Lit>>>(learnt->literals->size());
				seenList->insert(seenList->begin(), learnt->literals->begin(), learnt->literals->end());
				//simplify learnt clause
				//Here is still an error!!!!!!!
				for (int m = 0; m < learnt->literals->size() - 1; ++m)
				{
					shared_ptr<Lit> l = learnt->literals->at(m);
					//Ignore Literals without reason
					if (l->var->getClause() == nullptr)
					{
						continue;
					}
					else
					{
						//Check whether reason for current literal is already in learnt -> remove l
						shared_ptr<Clause> re = l->var->getClause();
						bool found = false;

						for (shared_ptr<Lit> rel : *(re->literals))
						{
							if (!rel->var->seen && (rel->var->decisionLevel != decisionLevel->at(0)))
							{
								found = true;
								break;
							}
						}

						if (!found)
						{
							learnt->literals->erase(learnt->literals->begin() + (m--));
						}
					}
				}
				//Reset Seen
				for (shared_ptr<Lit> l : *seenList)
				{
					l->var->seen = false;
				}

#ifdef CNSatDebug
				cout << "\nLearned " << endl;
				learnt->print();
				//This Stuff checks whether learnt is already in learntClauses -> throws exception
				/*	bool blub=false;
				 for(int r=0; r<satClauses->size(); r++) {
				 bool nof=true;
				 if(satClauses[r]->literals->size() != learnt->literals->size()) continue;
				 for(shared_ptr<Lit> l in learnt->literals) {
				 bool foundshared_ptr<Lit> = false;
				 for(shared_ptr<Lit> m in satClauses[r]->literals) {
				 if(l->var == (m->var) && l->sign == m->sign) {
				 foundshared_ptr<Lit> = true;
				 break;
				 }
				 }
				 nof &= foundLit;

				 if (false == nof) break;
				 }
				 if(nof) blub=true;
				 }
				 if(blub) throw new Exception("jbjkjdasklfjklasdkgklsaglajgkl");*/
#endif
				//End Learn Clause

				//Find backtracklevel:

				shared_ptr<DecisionLevel> db;
				int i = 1, maxLitIndex;
				shared_ptr<Lit> changeLit;

				if (learnt->literals->size() == 1)
					db = this->decisionLevel->at(0);
				else
				{
					db = learnt->literals->at(0)->var->decisionLevel;
					maxLitIndex = 0;

					//Search newest decission, which affects a literal in learnt.
					for (i = 1; i < learnt->literals->size() - 1; ++i)
					{
						shared_ptr<Lit> l = learnt->literals->at(i);

						if (db->level < l->var->decisionLevel->level)
						{
							db = l->var->decisionLevel;
							maxLitIndex = i;
						}
					}

					changeLit = learnt->literals->at(0);
					learnt->literals->at(0) = learnt->literals->at(maxLitIndex);
					learnt->literals->at(maxLitIndex) = changeLit;
				}

#ifdef CNSatDebug
				cout << "Backtracking from " << this->decisionLevel->at(this->decisionLevel->size() - 1)->level
				<< " to " << db->level << endl;
#endif

				//Backtrack to db
				backTrack(db);

				//Add learnt clause: Unit Clauses have to be satisfied otherwise: -> UNSAT
				bool solvable = this->addSATClause(learnt);
				if (!solvable)
				{ // TODO can be removed once bugfree
					cout << "Error on insert learned clause" << endl;
					return false;
				}

				/*if(db==DecisionLevel->at(0) && learnt->literals->size()>1) {
				 Console.WriteLine("Reached decision level 0");
				 return false;
				 }*/

				if (learnt->literals->size() == 1)
				{
					//decisions->at(0)->assignment = learnt->literals->at(0)->sign;
					learnt->literals->at(0)->var->assignment = learnt->literals->at(0)->sign;
					learnt->literals->at(0)->var->setClause(nullptr);
				}

				//Switch assignment of UIP to satisfy learnt
				if (learnt->literals->size() > 1)
				{
					//
					shared_ptr<DecisionLevel> d = make_shared<DecisionLevel>(decisions->size());
					//decisionLevel->push_back(d);

					//Set Learnt as Reason for UIP
					shared_ptr<Lit> l = learnt->literals->at(learnt->literals->size() - 1);
					l->var->assignment = l->sign;
					learnt->satisfied = true;
					l->var->decisionLevel = this->decisionLevel->at(this->decisionLevel->size() - 1);
					l->var->getClause() = learnt;
					decisions->push_back(l->var);
				}

#ifdef CNSatDebug
				this->printAssignments();
//				Console.ReadLine(); XXX ?!?!?
#endif
				return true;
			}

			void CNSat::backTrack(shared_ptr<DecisionLevel> db)
			{
				//TODO make this more efficient (linked list?)
				recentBacktrack = true;
				auto iter = find(decisionLevel->begin(), decisionLevel->end(), db);
				int ndbidx = distance(decisionLevel->begin(), iter) + 1;
				if (ndbidx >= decisionLevel->size())
					return;
				db = decisionLevel->at(ndbidx);
				for (int j = db->level; j < decisions->size(); j++)
				{
					decisions->at(j)->assignment = Assignment::UNASSIGNED;
					decisions->at(j)->setClause(nullptr);
					//this is expensive
					for (shared_ptr<Watcher> wa : *(decisions->at(j)->watchList))
					{
						wa->clause->satisfied = wa->clause->watcher->at(0)->lit->satisfied()
								|| wa->clause->watcher->at(1)->lit->satisfied();
						//wa->clause->satisfied = false; //this should take other watcher into account
					}
				}
				decisions->erase(decisions->begin() + db->level, decisions->begin() + (decisions->size() - db->level));

				//int i = decisionLevel.IndexOf(db);
				int i = ndbidx;
				i = std::max(1, i);
				decisionLevel->erase(decisionLevel->begin() + i, decisionLevel->begin() + (decisionLevel->size() - i));
			}

			void CNSat::backTrack(int decission)
			{
				this->recentBacktrack = true;
				if (decission < 0)
				{
					return;
				}
				if (this->decisionLevel->size() < 2)
				{
					shared_ptr<DecisionLevel> dl = make_shared<DecisionLevel>(decission);
					this->decisionLevel->push_back(dl);
				}

				decisionLevel->at(1)->level = decission;

				for (int j = decisionLevel->at(1)->level; j < this->decisions->size(); ++j)
				{
					decisions->at(j)->assignment = cnsat::Assignment::UNASSIGNED;
					decisions->at(j)->setClause(nullptr);
					decisions->at(j)->locked = false;
					for (shared_ptr<Watcher> wa : *(decisions->at(j)->watchList))
					{
						wa->clause->satisfied = false;
					}
				}
				if (decisionLevel->at(1)->level < decisions->size())
				{
					auto first = decisions->begin() + decisionLevel->at(1)->level;
					auto last = decisions->begin() + (decisions->size() - decisionLevel->at(1)->level);
					decisions->erase(first, last);
				}

				decisionLevel->erase(decisionLevel->begin() + 1, decisionLevel->begin() + (decisionLevel->size() - 1));
				decisionLevelNull->level = decisions->size();
			}

			void CNSat::emptyClauseList(shared_ptr<vector<shared_ptr<Clause>> > list)
			{
				for (shared_ptr<Clause> c : *list)
				{
					c->watcher->at(0)->lit->var->watchList->erase(c->watcher->begin());
					c->watcher->at(0)->lit->variableCount--;
					c->watcher->at(1)->lit->var->watchList->erase(c->watcher->begin() + 1);
					c->watcher->at(1)->lit->variableCount--;
				}
				list->size();
			}

			bool CNSat::solutionInsideRange(shared_ptr<vector<double> > solution,
											shared_ptr<vector<shared_ptr<vector<double> > > > range)
			{
				for (int i = 0; i < solution->size(); i++)
				{
					double val = solution->at(i);
					if (val < range->at(i)->at(0) || val > range->at(i)->at(1))
					{
						return false;
					}
				}
				return true;
			}

			bool CNSat::varAssignmentInsideRange(shared_ptr<Var> v,
													shared_ptr<vector<shared_ptr<vector<double> > > > range)
			{
				shared_ptr<vector<shared_ptr<vector<double>> > > litrange = nullptr;
				if (v->assignment == Assignment::TRUE)
					litrange = v->positiveRanges;
				else
					litrange = v->negativeRanges;
				for (int i = 0; i < litrange->size(); i++)
				{
					double min = litrange->at(i)->at(0);
					double max = litrange->at(i)->at(1);
					if (min < range->at(i)->at(0) || max > range->at(i)->at(1))
					{
						return false;
					}
				}
				return true;
			}

			bool CNSat::assignmentInsideRange(shared_ptr<DecisionLevel> dl,
												shared_ptr<vector<shared_ptr<vector<double> > > > range)
			{
				for (int i = dl->level; i < decisions->size(); i++)
				{
					if (!varAssignmentInsideRange(decisions->at(i), range))
						return false;
				}
				return true;
			}

			void CNSat::printAssignments()
			{
				for (shared_ptr<Var> v : *variables)
				{
					v->print();
					cout << " ";
				}
				cout << endl;
			}

		} /* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */
