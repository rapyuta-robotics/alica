#pragma once

#include "UtilityInterval.h"
#include "engine/RunningPlan.h"
#include "engine/TaskRoleStruct.h"

#include <map>
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
    UtilityFunction();
    UtilityFunction(double priorityWeight, double similarityWeight, const Plan* plan);
    ~UtilityFunction();
    const std::vector<std::unique_ptr<USummand>>& getUtilSummands() const { return _utilSummands; };
    std::vector<std::unique_ptr<USummand>>& editUtilSummands() { return _utilSummands; };

    // double eval(const RunningPlan* newRp, const RunningPlan* oldRp) const;
    UtilityInterval eval(const PartialAssignment* newAss, const Assignment* oldAss) const;
    // void updateAssignment(IAssignment* newAss, const Assignment* oldAss);
    void cacheEvalData();
    void init(AlicaEngine* ae);

    static void initDataStructures(AlicaEngine* ae);

    const Plan* getPlan() const { return _plan; }
    // const std::map<TaskRoleStruct, double>& getPriorityMartix() const { return priorityMatrix; }

    const double DIFFERENCETHRESHOLD = 0.0001; // Max difference for the same result

private:
    friend std::stringstream& operator<<(std::stringstream& ss, const UtilityFunction& uf);
    UtilityInterval getPriorityResult(IAssignment ass) const;
    UtilityInterval getSimilarity(IAssignment newAss, const Assignment* oldAss) const;

    const Plan* _plan;
    std::vector<std::unique_ptr<USummand>> _utilSummands;

    // For default priority based utility summand (which is integrated in every UF)
    std::map<TaskRoleStruct, double> _priorityMatrix;
    std::map<int64_t, double> _roleHighestPriorityMap;
    // For default similarity based utility summand (which is integrated in every UF)
    double _priorityWeight;
    double _similarityWeight;

    AlicaEngine* _ae;
    IRoleAssignment* _ra;
};

std::stringstream& operator<<(std::stringstream& ss, const UtilityFunction& uf);

} /* namespace alica */
