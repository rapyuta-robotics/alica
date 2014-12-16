/*
 * FormulaTransform.h
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#ifndef FORMULATRANSFORM_H_
#define FORMULATRANSFORM_H_

#include <AutoDiff.h>

#include <memory>
#include <vector>
#include <map>

using namespace std;
using namespace autodiff;

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{
			class Clause;
			class CNSat;
			class Lit;
			class TermEquality;
			class Var;

			class FormulaTransform
			{
			public:
				FormulaTransform();
				virtual ~FormulaTransform();

				void reset();
				shared_ptr<vector<shared_ptr<Clause>> > transformToCNF(shared_ptr<Term> formula,
																		shared_ptr<CNSat> solver);
				shared_ptr<Var> getAtoms(int term_id);
				int getAtomOccurrence();

			protected:
				map<int, shared_ptr<Var>> atoms;
				int atomOccurrence;

				shared_ptr<CNSat> solver;

				shared_ptr<TermEquality> te;

				void doTransform(shared_ptr<vector<shared_ptr<Clause>> > clauses);
				void performStep(shared_ptr<Clause> c, shared_ptr<Lit> lit, shared_ptr<Clause> newClause1,
									shared_ptr<Clause> newClause2);
				bool tryGetVar(shared_ptr<Term> t, shared_ptr<Var> v);
			};

		}
	/* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */

#endif /* FORMULATRANSFORM_H_ */
