/*
 * Clause.h
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#ifndef CLAUSE_H_
#define CLAUSE_H_

#include "types/Lit.h"

#include <vector>
#include <memory>

using namespace std;

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{
//			class Lit;
			class Var;
			class Watcher;

			class Clause
			{
			public:
				Clause();
				virtual ~Clause();

				void addChecked(shared_ptr<Lit> l);
				shared_ptr<Clause> clone();
				void add(shared_ptr<Lit> l);
				int avgActivity();
				bool checkSatisfied();

				static bool compareTo(shared_ptr<Clause> ep1, shared_ptr<Clause> ep2);

				void print();

				bool isTautologic;
				bool isFinished;

				bool satisfied;
				shared_ptr<vector<Watcher*> > watcher;
				shared_ptr<Var> lastModVar;
				int activity;
				shared_ptr<vector<shared_ptr<Lit>>> literals;
			};

		}
	/* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */

#endif /* CLAUSE_H_ */
