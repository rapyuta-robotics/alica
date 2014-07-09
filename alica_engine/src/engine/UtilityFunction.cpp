/*
 * UtilityFunction.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#include "engine/UtilityFunction.h"
#include "engine/TaskRoleStruct.h"
#include "engine/UtilityInterval.h"

namespace alica
{

	UtilityFunction::UtilityFunction(string name, list<USummand*> utilSummands, double priorityWeight, double similarityWeight, Plan* plan)
	{
		this->lookupStruct = new TaskRoleStruct(0,0);
		this->priResult = new UtilityInterval(0.0,0.0);
		this->simUI = new UtilityInterval(0.0,0.0);

	}

	UtilityFunction::~UtilityFunction()
	{
	}

	list<USummand*> UtilityFunction::getUtilSummands()
	{
		return utilSummands;
	}

	void UtilityFunction::setUtilSummands(list<USummand*> utilSummands)
	{
		this->utilSummands = utilSummands;
	}

	Plan* UtilityFunction::getPlan()
	{
		return plan;
	}

	map<TaskRoleStruct*, double> UtilityFunction::getPriorityMartix()
	{
		return priorityMartix;
	}

} /* namespace alica */
