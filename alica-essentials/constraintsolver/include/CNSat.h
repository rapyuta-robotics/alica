/*
 * CNSat.h
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#ifndef CNSAT_H_
#define CNSAT_H_

#include "types/Assignment.h"

#include <vector>
#include <memory>

using namespace std;

namespace alica
{
	class IAlicaClock;

	namespace reasoner
	{
		class CNSMTGSolver;

		namespace cnsat
		{
			class Clause;
			class DecisionLevel;
			class Var;

			class CNSat : public enable_shared_from_this<CNSat>
			{
			public:
				CNSat();
				virtual ~CNSat();

				void readFromCNFFile(string path);

				shared_ptr<Var> newVar();
				bool addBasicClause(shared_ptr<Clause> c);
				void emptySATClause();
				void emptyTClause();
				void resetVariables();
				bool addSATClause(shared_ptr<Clause> c);
				bool addTClause(shared_ptr<Clause> c);
				bool preAddIUnitClause(shared_ptr<Var> v, Assignment ass);
				bool addIClause(shared_ptr<Clause> c);
				void init();
				bool solve();
				void reduceDB(int num);
				shared_ptr<Clause> propagate();
				bool resolveConflict(shared_ptr<Clause> c);
				void backTrack(shared_ptr<DecisionLevel> db);
				void backTrack(int decission);
				void printStatistics();
				void printAssignments();

				bool useIntervalProp;
				weak_ptr<CNSMTGSolver> cnsmtGSolver;

				shared_ptr<vector<shared_ptr<Clause> >> clauses;
				shared_ptr<vector<shared_ptr<Clause> >> satClauses;
				shared_ptr<vector<shared_ptr<Clause> >> tClauses;
				shared_ptr<vector<shared_ptr<Clause> >> iClauses;
				shared_ptr<vector<shared_ptr<Var> >> variables;
				shared_ptr<vector<shared_ptr<Var> >> decisions;
				shared_ptr<vector<shared_ptr<DecisionLevel> >> decisionLevel;

				int unitDecissions;

			protected:
				void emptyClauseList(shared_ptr<vector<shared_ptr<Clause>>> list);
				bool solutionInsideRange(shared_ptr<vector<double>> solution,
											shared_ptr<vector<shared_ptr<vector<double>> > > range);
				bool varAssignmentInsideRange(shared_ptr<Var> v,
												shared_ptr<vector<shared_ptr<vector<double>> > > range);
				bool assignmentInsideRange(shared_ptr<DecisionLevel> dl,
											shared_ptr<vector<shared_ptr<vector<double>> > > range);

				int conflictCount = 0;
				int decisionCount = 0;
				int learnedCount = 0;
				int learntNum;
				int restartCount = 0;
				shared_ptr<DecisionLevel> decisionLevelNull;
				bool recentBacktrack = false;

				IAlicaClock* alicaClock;

			};
		} /* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */

#endif /* CNSAT_H_ */
