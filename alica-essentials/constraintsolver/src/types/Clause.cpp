/*
 * Clause.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#include "types/Clause.h"

#include "types/Lit.h"
#include "types/Var.h"
#include "types/Watcher.h"

#include <iostream>

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{

			Clause::Clause()
			{
				activity = 0;
				satisfied = false;
				isFinished = false;
				isTautologic = false;
				watcher = make_shared<vector<Watcher*>>(2);
				literals = make_shared<vector<shared_ptr<Lit>>>();
			}

			Clause::~Clause()
			{
				for (Watcher* w : *watcher)
				{
//					w->lit->var->watchList // TODO
					delete w;
				}
			}

			void Clause::addChecked(shared_ptr<Lit> l)
			{
				bool found = false;
				if (l->isTemporary)
				{
					for (int i = 0; i < literals->size(); ++i)
					{
						if (literals->at(i)->isTemporary && l->atom == literals->at(i)->atom)
						{
							found = true;
							break;
						}
					}
					if (!found)
					{
						literals->push_back(l);
					}
					return;
				}
				for (int i = 0; i < literals->size(); ++i)
				{
					if (l->atom == literals->at(i)->atom)
					{
						found = true;
						if (l->sign != literals->at(i)->sign)
						{
							isTautologic = true;
							literals->clear();
						}
						break;
					}
				}
				if (!found)
				{
					literals->push_back(l);
				}
				return;
			}

			shared_ptr<Clause> Clause::clone()
			{
				shared_ptr<Clause> clone = make_shared<Clause>();
				//clone->literals->insert(clone->literals->end(), literals->begin(), literals->end());
				*clone->literals = *literals;
//				clone->literals = make_shared<vector<shared_ptr<Lit>>>();
//				for (shared_ptr<Lit> l : *literals) {
//					clone->literals->push_back(l);
//				}
				clone->isFinished = isFinished;
				clone->isTautologic = isTautologic;
				return clone;
			}

			void Clause::add(shared_ptr<Lit> l)
			{
				literals->push_back(l);
			}

			int Clause::avgActivity()
			{
				int ret = 0;
				for (shared_ptr<Lit> l : *literals)
				{
					ret += l->var->activity;
				}
				return ret / literals->size();
			}

			bool Clause::checkSatisfied()
			{
				for (shared_ptr<Lit> l : *literals)
				{
					if (l->var->assignment == l->sign)
					{
						return true;
					}
				}
				return false;
			}

			bool Clause::compareTo(shared_ptr<Clause> ep1, shared_ptr<Clause> ep2)
			{
				return ep1->activity > ep2->activity;
			}

			void Clause::print()
			{
				for (shared_ptr<Lit> l : *literals)
				{
					if (l->sign == Assignment::FALSE)
					{
						cout << "-";
					}
					if (l->sign == Assignment::TRUE)
					{
						cout << "+";
					}
					if (l->sign == Assignment::UNASSIGNED)
					{
						cout << "#";
					}
					cout << l->var->index;
					if ((watcher->at(0) && watcher->at(0)->lit == l) || (watcher->at(1) && watcher->at(1)->lit == l))
					{
						cout << "w";
					}
					cout << " ";
				}
				cout << endl;
			}
		} /* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */
