/*
 * PlanningProblem.cpp
 *
 *  Created on: Jul 1, 2014
 *      Author: Paul Panin
 */

#include "engine/model/PlanningProblem.h"

namespace alica {

PlanningProblem::PlanningProblem() {
    this->runtimeCondition = nullptr;
    this->preCondition = nullptr;
    this->alternativePlan = nullptr;
    this->planningType = PlanningType::None;
    this->updateRate = 0;
    this->postCondition = nullptr;
    this->waitPlan = nullptr;
    this->distributeProblem = false;
}

PlanningProblem::~PlanningProblem() {}

const Plan* PlanningProblem::getAlternativePlan() const {
    return alternativePlan;
}

void PlanningProblem::setAlternativePlan(Plan* alternativePlan) {
    this->alternativePlan = alternativePlan;
}

bool PlanningProblem::isDistributeProblem() const {
    return distributeProblem;
}

void PlanningProblem::setDistributeProblem(bool distributeProblem) {
    this->distributeProblem = distributeProblem;
}


PlanningType PlanningProblem::getPlanningType() const {
    return planningType;
}

void PlanningProblem::setPlanningType(PlanningType planningType) {
    this->planningType = planningType;
}

list<AbstractPlan*>& PlanningProblem::getPlans() {
    return plans;
}

void PlanningProblem::setPlans(list<AbstractPlan*>& plans) {
    this->plans = plans;
}

const PostCondition* PlanningProblem::getPostCondition() const {
    return postCondition;
}

void PlanningProblem::setPostCondition(PostCondition* postCondition) {
    this->postCondition = postCondition;
}

const PreCondition* PlanningProblem::getPreCondition() const {
    return preCondition;
}

void PlanningProblem::setPreCondition(PreCondition* preCondition) {
    this->preCondition = preCondition;
}

const string& PlanningProblem::getRequirements() const {
    return requirements;
}

void PlanningProblem::setRequirements(const string& requirements) {
    this->requirements = requirements;
}

const RuntimeCondition* PlanningProblem::getRuntimeCondition() const {
    return runtimeCondition;
}

void PlanningProblem::setRuntimeCondition(RuntimeCondition* runtimeCondition) {
    this->runtimeCondition = runtimeCondition;
}

int PlanningProblem::getUpdateRate() const {
    return updateRate;
}

void PlanningProblem::setUpdateRate(int updateRate) {
    this->updateRate = updateRate;
}

const Plan* PlanningProblem::getWaitPlan() const {
    return waitPlan;
}

void PlanningProblem::setWaitPlan(Plan* waitPlan) {
    this->waitPlan = waitPlan;
}

}  // namespace alica
