/*
 * ConstraintStore.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: Paul Panin
 */

#include "engine/constraintmodul/ConstraintStore.h"

namespace alica
{

	ConstraintStore::ConstraintStore(RunningPlan* rp)
	{
		this->rp = rp;
		this->activeConitions = unordered_set<Condition*>();
		this->activeVariables = map<Variable*, list<Condition*> >();
		//TODO:
	}

	ConstraintStore::~ConstraintStore()
	{
		//TODO:
	}

	void ConstraintStore::addCondition(Condition* con)
	{
		//TODO:
	}
	void ConstraintStore::clear()
	{
		//TODO:
	}

} /* namespace supplementary */
