#pragma once

//#define Q_DEBUG

#include "engine/AlicaEngine.h"
#include "engine/BasicBehaviour.h"
#include "engine/AlicaClock.h"
#include "engine/RunningPlan.h"
#include "engine/TeamObserver.h"
#include "engine/constraintmodul/ConditionStore.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/constraintmodul/IVariableSyncModule.h"
#include "engine/constraintmodul/ProblemDescriptor.h"
#include "engine/constraintmodul/ProblemPart.h"
#include "engine/constraintmodul/SolverTerm.h"
#include "engine/constraintmodul/SolverVariable.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/model/Condition.h"
#include "engine/model/Parametrisation.h"
#include "engine/model/PlanType.h"
#include "engine/model/State.h"
#include "engine/model/Variable.h"

#include <map>
#include <memory>
#include <vector>

namespace alica {
class AlicaEngine;
class ProblemPart;
class RunningPlan;
class AlicaClock;
class BasicBehaviour;
class ISolver;

/**
 * Internal class to deal with bindings in states and plantypes
 */
class UniqueVarStore {
public:
    UniqueVarStore();

    void clear();
    void add(const Variable* v);
    const Variable* getRep(const Variable* v);
    void addVarTo(const Variable* representing, const Variable* toAdd);
    VariableSet getAllRep() const;
    int getIndexOf(const Variable* v) const;
    friend std::ostream& operator<<(std::ostream& os, const UniqueVarStore& store) {
        os << "UniqueVarStore: " << std::endl;
        // write obj to stream
        for (auto& variableList : store.store) {
            os << "VariableList: ";
            for (auto& variable : variableList) {
                os << *variable << ", ";
            }
            os << std::endl;
        }
        return os;
    }

private:
    /**
     *  Each inner list of variables is sorted from variables of the top most plan to variables of the deepest plan.
     *  Therefore, the first element is always the variable in the top most plan, where this variable occurs.
     */
    std::vector<VariableSet> store;
};

/**
 * Encapsulates queries to variables (which are associated with specific solvers).
 */
class Query {
public:
    Query();

    void addStaticVariable(const Variable* v);
    void addDomainVariable(const supplementary::AgentID* robot, const std::string& ident, AlicaEngine* ae);
    void clearDomainVariables();
    void clearStaticVariables();
    bool existsSolution(int solverType, std::shared_ptr<RunningPlan> rp);

    template <class T>
    bool getSolution(int solverType, std::shared_ptr<RunningPlan> rp, vector<T>& result);

    const VariableSet& getRelevantStaticVariables() const { return relevantStaticVariables; }
    const VariableSet& getRelevantDomainVariables() const { return relevantDomainVariables; }
    void setRelevantStaticVariables(const VariableSet& value);
    void setRelevantDomainVariables(const VariableSet& value);

    void addProblemParts(std::vector<std::shared_ptr<ProblemPart>>& l);

    std::shared_ptr<UniqueVarStore> getUniqueVariableStore(); /*< for testing only!!! */

private:
    bool collectProblemStatement(std::shared_ptr<RunningPlan> rp, ISolver* solver,
            std::vector<std::shared_ptr<ProblemDescriptor>>& cds, VariableSet& relevantVariables, int& domOffset);

    std::shared_ptr<UniqueVarStore> uniqueVarStore;
    VariableSet queriedStaticVariables;
    VariableSet queriedDomainVariables;
    std::vector<shared_ptr<ProblemPart>> problemParts;

    VariableSet relevantStaticVariables;
    VariableSet relevantDomainVariables;
};

template <class T>
bool Query::getSolution(int solverType, std::shared_ptr<RunningPlan> rp, std::vector<T>& result) {
    result.clear();

    // Collect the complete problem specification
    vector<shared_ptr<ProblemDescriptor>> cds;
    VariableSet relevantVariables;
    int domOffset;
    ISolver* solver = rp->getAlicaEngine()->getSolver(solverType);
    if (solver == nullptr) {
        std::cerr << "Query::getSolution: The engine does not have a suitable solver for the given type available."
                  << std::endl;
        return false;
    }

    if (!this->collectProblemStatement(rp, solver, cds, relevantVariables, domOffset)) {
        return false;
    }

#ifdef Q_DEBUG
    std::cout << "Query: " << (*this->uniqueVarStore) << std::endl;
#endif

    // the result of the solver (including all relevant variables)
    vector<void*> solverResult;
    // let the solver solve the problem
    bool ret = solver->getSolution(relevantVariables, cds, solverResult);

    if (solverResult.size() > 0) {
        // TODO: currently only synch variables if the result/their value is a double
        if (typeid(T) == typeid(double) && ret) {
            for (int i = 0; i < solverResult.size(); i++) {
                uint8_t* tmp = ((uint8_t*) solverResult.at(i));
                shared_ptr<vector<uint8_t>> result = make_shared<vector<uint8_t>>(sizeof(T));
                // If you have an Segfault/Error here you solver does not return what you are querying! ;)
                for (int s = 0; s < sizeof(T); s++) {
                    result->at(s) = *tmp;
                    tmp++;
                }

                solver->getAlicaEngine()->getResultStore()->postResult(relevantVariables.at(i)->getId(), result);
            }
        }

        // create a result vector that is filtered by the queried variables
        for (auto& staticVariable : queriedStaticVariables) {
            result.push_back(*((T*) solverResult.at(uniqueVarStore->getIndexOf(staticVariable))));
        }

        for (int i = 0; i < queriedDomainVariables.size(); ++i) {
            for (int j = 0; j < relevantDomainVariables.size(); ++j) {
                if (relevantDomainVariables[j] == queriedDomainVariables[i]) {
                    result.push_back(*((T*) solverResult.at(domOffset + j)));
                    break;
                }
            }
        }
    }

    // deallocate "solverResults" (they where copied into "result")
    for (int i = 0; i < solverResult.size(); i++) {
        delete (T*) solverResult.at(i);
    }

    return ret;
}
} /* namespace alica */
