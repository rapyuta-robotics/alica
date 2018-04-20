/*
 * PlanningProblem.cpp
 *
 *  Created on: Jul 1, 2014
 *      Author: Paul Panin
 */

#include "engine/model/PlanningProblem.h"

namespace alica {

PlanningProblem::PlanningProblem() 
    : _runtimeCondition(nullptr)
    , _preCondition(nullptr)
    , _alternativePlan(nullptr)
    , _planningType(PlanningType::None)
    , _updateRate(0)
    , _postCondition(nullptr)
    , _waitPlan(nullptr)
    , _distributeProblem(false)
{}

PlanningProblem::~PlanningProblem() {}

void PlanningProblem::setAlternativePlan(const Plan* alternativePlan) {
    _alternativePlan = alternativePlan;
}

void PlanningProblem::setDistributeProblem(bool distributeProblem) {
    _distributeProblem = distributeProblem;
}


void PlanningProblem::setPlanningType(PlanningType planningType) {
    _planningType = planningType;
}

void PlanningProblem::setPlans(const AbstractPlanGrp& plans) {
    _plans = plans;
}

void PlanningProblem::setPostCondition(PostCondition* postCondition) {
    _postCondition = postCondition;
}

void PlanningProblem::setPreCondition(PreCondition* preCondition) {
    _preCondition = preCondition;
}

void PlanningProblem::setRequirements(const std::string& requirements) {
    _requirements = requirements;
}

void PlanningProblem::setRuntimeCondition(RuntimeCondition* runtimeCondition) {
    _runtimeCondition = runtimeCondition;
}

void PlanningProblem::setUpdateRate(int updateRate) {
    _updateRate = updateRate;
}

void PlanningProblem::setWaitPlan(const Plan* waitPlan) {
    _waitPlan = waitPlan;
}

}  // namespace alica
