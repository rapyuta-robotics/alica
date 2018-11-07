/*
 * PlanningProblem.h
 *
 *  Created on: Jul 1, 2014
 *      Author: Paul Panin
 */

#ifndef PLANNINGPROBLEM_H_
#define PLANNINGPROBLEM_H_

#include "AbstractPlan.h"
#include "PlanningType.h"
#include "engine/Types.h"

#include <string>

namespace alica
{
class Condition;
class PreCondition;
class PostCondition;
class RuntimeCondition;
class ModelFactory;

/**
 * An ALICA planningProblem
 */
class PlanningProblem : public AbstractPlan
{
public:
    PlanningProblem();
    virtual ~PlanningProblem();
    const Plan* getAlternativePlan() const { return _alternativePlan; }
    const Plan* getWaitPlan() const { return _waitPlan; }

    PlanningType getPlanningType() const { return _planningType; }
    const AbstractPlanGrp& getPlans() const { return _plans; }

    const PostCondition* getPostCondition() const { return _postCondition; }
    const PreCondition* getPreCondition() const { return _preCondition; }
    const RuntimeCondition* getRuntimeCondition() const { return _runtimeCondition; }

    const std::string& getRequirements() const { return _requirements; }

    bool isDistributeProblem() const { return _distributeProblem; }
    int getUpdateRate() const { return _updateRate; }

private:
    friend ModelFactory;
    void setAlternativePlan(const Plan* alternativePlan);
    void setDistributeProblem(bool distributeProblem);
    void setPlanningType(PlanningType planningType);
    void setPlans(const AbstractPlanGrp& plans);
    void setPostCondition(PostCondition* postCondition);
    void setPreCondition(PreCondition* preCondition);
    void setRequirements(const std::string& requirements);
    void setRuntimeCondition(RuntimeCondition* runtimeCondition);
    void setUpdateRate(int updateRate);
    void setWaitPlan(const Plan* waitPlan);

    AbstractPlanGrp _plans;
    const Plan* _alternativePlan;
    const Plan* _waitPlan;
    PreCondition* _preCondition;
    RuntimeCondition* _runtimeCondition;
    PostCondition* _postCondition;
    int _updateRate;
    bool _distributeProblem;
    PlanningType _planningType;
    std::string _requirements;
};

} // namespace alica

#endif /* PLANNINGPROBLEM_H_ */
