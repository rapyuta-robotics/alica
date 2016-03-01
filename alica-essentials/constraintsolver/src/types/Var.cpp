/*
 * Var.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#include "types/Var.h"

#include "types/Clause.h"
#include "types/DecisionLevel.h"
#include "types/Lit.h"
#include "types/Watcher.h"

#include <iostream>

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{

			Var::Var(int index, bool prefSign)
			{
				this->index = index;
				this->assignment = Assignment::UNASSIGNED;
				this->locked = false;
				this->preferedSign = prefSign;
				this->activity = 0;

				positiveRanges = nullptr;
				negativeRanges = nullptr;

				watchList = make_shared<vector<Watcher*> >();
			}

			Var::~Var()
			{

			}

			shared_ptr<Clause> Var::getReason()
			{
				return reason;
			}

			void Var::setReason(shared_ptr<Clause> reason)
			{
				if (reason && reason != this->reason)
				{
					for (shared_ptr<Lit> l : *reason->literals)
					{
						if (l->var->assignment == Assignment::UNASSIGNED)
						{
							cout << this->toString() << " " << l->var->toString();
							reason->print();
							cerr << "!!!!!";
							throw "!!!!!";
						}
					}
				}
				this->reason = reason;
			}

			void Var::reset()
			{
				if (locked)
				{
					return;
				}
				this->reason = nullptr;
				this->seen = false;

				cout << "Var::reset() unass" << endl;
				this->assignment = Assignment::UNASSIGNED;

				this->activity = 0;
			}

			void Var::print()
			{
				if (this->assignment == Assignment::FALSE)
				{
					cout << "-";
				}
				else if (this->assignment == Assignment::UNASSIGNED)
				{
					cout << "o";
				}
				else
				{
					cout << "+";
				}
				cout << this->index;
			}

			string Var::toString()
			{
				string str;
				if (this->assignment == Assignment::FALSE)
					str.append("-");
				else if (this->assignment == Assignment::TRUE)
					str.append("+");
				else
					str.append("o");
				str.append(to_string(this->index));
				return str;
			}

		} /* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */
