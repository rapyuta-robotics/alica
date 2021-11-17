/*
 * CNSat.h
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#ifndef CNSAT_H_
#define CNSAT_H_

#include "types/Assignment.h"
#include <engine/AlicaClock.h>
#include <memory>
#include <vector>

namespace alica
{
class AlicaClock;

namespace reasoner
{
class CNSMTGSolver;

namespace cnsat
{
class Clause;
class DecisionLevel;
class Var;

class CNSat
{
  public:
    CNSat();
    virtual ~CNSat();

    void readFromCNFFile(std::string path);

    std::shared_ptr<Var> newVar();
    bool addBasicClause(std::shared_ptr<Clause> c);
    void emptySATClause();
    void emptyTClause();
    void resetVariables();
    bool addSATClause(std::shared_ptr<Clause> c);
    bool addTClause(std::shared_ptr<Clause> c);
    bool preAddIUnitClause(std::shared_ptr<Var> v, Assignment ass);
    bool addIClause(std::shared_ptr<Clause> c);
    void init();
    bool solve(AlicaTime until, CNSMTGSolver* callbackSolver = nullptr);
    void reduceDB(int num);
    std::shared_ptr<Clause> propagate();
    bool resolveConflict(std::shared_ptr<Clause> c);
    void backTrack(std::shared_ptr<DecisionLevel> db);
    void backTrack(int decission);
    void printStatistics();
    void printAssignments();

    void removeRangeOfDecisions(int index, int count);

    bool useIntervalProp;

    std::shared_ptr<std::vector<std::shared_ptr<Clause>>> clauses;
    std::shared_ptr<std::vector<std::shared_ptr<Clause>>> satClauses;
    std::shared_ptr<std::vector<std::shared_ptr<Clause>>> tClauses;
    std::shared_ptr<std::vector<std::shared_ptr<Clause>>> iClauses;
    std::shared_ptr<std::vector<std::shared_ptr<Var>>> variables;
    std::shared_ptr<std::vector<std::shared_ptr<Var>>> decisions;
    std::shared_ptr<std::vector<std::shared_ptr<DecisionLevel>>> decisionLevel;
    CNSMTGSolver* cnsmtGSolver;
    int unitDecissions;

  protected:
    void emptyClauseList(std::shared_ptr<std::vector<std::shared_ptr<Clause>>> list);
    bool solutionInsideRange(std::shared_ptr<std::vector<double>> solution, std::shared_ptr<std::vector<std::shared_ptr<std::vector<double>>>> range);
    bool varAssignmentInsideRange(std::shared_ptr<Var> v, std::shared_ptr<std::vector<std::shared_ptr<std::vector<double>>>> range);
    bool assignmentInsideRange(std::shared_ptr<DecisionLevel> dl, std::shared_ptr<std::vector<std::shared_ptr<std::vector<double>>>> range);

    int conflictCount = 0;
    int decisionCount = 0;
    int learnedCount = 0;
    int learntNum;
    int restartCount = 0;
    std::shared_ptr<DecisionLevel> decisionLevelNull;
    bool recentBacktrack = false;

    AlicaClock alicaClock;
};
} /* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */

#endif /* CNSAT_H_ */
