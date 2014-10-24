/*
 * PlanningProblem.h
 *
 *  Created on: Jul 1, 2014
 *      Author: Paul Panin
 */

#ifndef PLANNINGPROBLEM_H_
#define PLANNINGPROBLEM_H_


#include <list>
#include "Plan.h"
#include "PreCondition.h"
#include "PostCondition.h"
#include "RuntimeCondition.h"
#include "PlanningType.h"
#include "AlicaElement.h"
#include "AbstractPlan.h"
#include <string>
#include <iostream>

using namespace std;

namespace alica
{

	/**
	 * An ALICA planningProblem
	 */
	class PlanningProblem : public AbstractPlan
	{
	public:
		PlanningProblem();
		virtual ~PlanningProblem();
		const Plan* getAlternativePlan() const;
		void setAlternativePlan(Plan* alternativePlan);
		bool isDistributeProblem() const;
		void setDistributeProblem(bool distributeProblem);
		const string& getFileName() const;
		void setFileName(const string& fileName);
		PlanningType getPlanningType() const;
		void setPlanningType(PlanningType planningType);
		list<AbstractPlan*>& getPlans();
		void setPlans(list<AbstractPlan*>& plans);
		const PostCondition* getPostCondition() const;
		void setPostCondition(PostCondition* postCondition);
		const PreCondition* getPreCondition() const;
		void setPreCondition( PreCondition* preCondition);
		const string& getRequirements() const;
		void setRequirements(const string& requirements);
		const RuntimeCondition* getRuntimeCondition() const;
		void setRuntimeCondition(RuntimeCondition* runtimeCondition);
		int getUpdateRate() const;
		void setUpdateRate(int updateRate);
		const Plan* getWaitPlan() const;
		void setWaitPlan(Plan* waitPlan);

		list<AbstractPlan*> plans;
		Plan* alternativePlan;
		Plan* waitPlan;
		int updateRate;
		bool distributeProblem;
		PlanningType planningType;
		string requirements;
		string fileName;
		PostCondition* postCondition;
		PreCondition* preCondition;
		RuntimeCondition* runtimeCondition;





	};

} /* namespace supplementary */

#endif /* PLANNINGPROBLEM_H_ */
