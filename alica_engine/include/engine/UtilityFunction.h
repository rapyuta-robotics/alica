/*
 * UtilityFunction.h
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef UTILITYFUNCTION_H_
#define UTILITYFUNCTION_H_
//#define UFDEBUG

#include <string>
#include <map>
#include <list>
#include <algorithm>
#include <vector>
#include <sstream>
#include <memory>
#include "UtilityInterval.h"
#include "engine/RunningPlan.h"

namespace alica {
class Plan;
class AlicaEngine;
class IRoleAssignment;
class USummand;
class IAssignment;
struct TaskRoleStruct;

class UtilityFunction {
public:
    UtilityFunction(const std::string& name, std::list<USummand*> utilSummands, double priorityWeight,
            double similarityWeight, const Plan* plan);
    virtual ~UtilityFunction();
    std::list<USummand*>& getUtilSummands();
    void setUtilSummands(std::list<USummand*> utilSummands);
    virtual double eval(std::shared_ptr<RunningPlan> newRp, std::shared_ptr<RunningPlan> oldRp);
    virtual UtilityInterval eval(IAssignment* newAss, IAssignment* oldAss);
    void updateAssignment(IAssignment* newAss, IAssignment* oldAss);
    void cacheEvalData();
    void init(AlicaEngine* ae);
    virtual std::pair<std::vector<double>, double>* differentiate(IAssignment* newAss);
    static void initDataStructures(AlicaEngine* ae);
    virtual std::string toString() const;
    const Plan* getPlan() const { return plan; }
    const std::map<TaskRoleStruct*, double>& getPriorityMartix() const;

    const double DIFFERENCETHRESHOLD = 0.0001;  // Max difference for the same result

protected:
    UtilityInterval getPriorityResult(IAssignment* ass);
    UtilityInterval getSimilarity(IAssignment* newAss, IAssignment* oldAss);

    const Plan* plan;

    std::string name = "DefaultUtilityFunction";
    // For default priority based utility summand (which is integrated in every UF)
    std::map<TaskRoleStruct*, double> priorityMartix;
    std::map<long, double> roleHighestPriorityMap;
    // For default similarity based utility summand (which is integrated in every UF)
    double priorityWeight;
    double similarityWeight;
    AlicaEngine* ae;
    IRoleAssignment* ra;
    // List of normal utility summands
    std::list<USummand*> utilSummands;
    TaskRoleStruct* lookupStruct;
    UtilityInterval priResult;

    UtilityInterval simUI;
};

} /* namespace alica */

#endif /* UTILITYFUNCTION_H_ */
