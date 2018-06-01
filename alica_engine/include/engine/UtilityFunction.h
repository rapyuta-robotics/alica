/*
 * UtilityFunction.h
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef UTILITYFUNCTION_H_
#define UTILITYFUNCTION_H_
//#define UFDEBUG

#include "UtilityInterval.h"
#include "engine/RunningPlan.h"
#include <algorithm>
#include <list>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace alica
{
class Plan;
class AlicaEngine;
class IRoleAssignment;
class USummand;
class IAssignment;
struct TaskRoleStruct;

class UtilityFunction
{
public:
    UtilityFunction(const std::string& name, std::list<USummand*> utilSummands, double priorityWeight, double similarityWeight, const Plan* plan);
    virtual ~UtilityFunction();
    std::list<USummand*>& getUtilSummands();
    void setUtilSummands(std::list<USummand*> utilSummands);
    virtual double eval(const RunningPlan* newRp, const RunningPlan* oldRp);
    virtual UtilityInterval eval(IAssignment* newAss, IAssignment* oldAss);
    void updateAssignment(IAssignment* newAss, const Assignment* oldAss);
    void cacheEvalData();
    void init(AlicaEngine* ae);
    virtual std::pair<std::vector<double>, double>* differentiate(IAssignment* newAss);
    static void initDataStructures(AlicaEngine* ae);
    virtual std::string toString() const;
    const Plan* getPlan() const { return plan; }
    const std::map<TaskRoleStruct, double>& getPriorityMartix() const { return priorityMatrix; }

    const double DIFFERENCETHRESHOLD = 0.0001; // Max difference for the same result

protected:
    UtilityInterval getPriorityResult(IAssignment* ass);
    UtilityInterval getSimilarity(const IAssignment* newAss, const Assignment* oldAss);

    const Plan* plan;

    std::string name = "DefaultUtilityFunction";
    // For default priority based utility summand (which is integrated in every UF)
    std::map<TaskRoleStruct, double> priorityMatrix;
    std::map<int64_t, double> roleHighestPriorityMap;
    // For default similarity based utility summand (which is integrated in every UF)
    double priorityWeight;
    double similarityWeight;
    AlicaEngine* ae;
    IRoleAssignment* ra;
    // List of normal utility summands
    std::list<USummand*> utilSummands;
    UtilityInterval priResult;

    UtilityInterval simUI;
};

} /* namespace alica */

#endif /* UTILITYFUNCTION_H_ */
