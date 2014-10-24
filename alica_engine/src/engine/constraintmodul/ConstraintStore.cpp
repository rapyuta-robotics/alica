/*
 * ConstraintStore.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: Paul Panin
 */

#include "engine/constraintmodul/ConstraintStore.h"

namespace alica
{

	/**
	 * Default constructor
	 * @param A RunningPlan
	 */
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

	/**
	 * Add a condition to the store
	 * @param con A Condition
	 */
	void ConstraintStore::addCondition(Condition* con)
	{
		//TODO:
	}

	/**
	 * Clear store, revoking all constraints
	 */
	void ConstraintStore::clear()
	{
		//TODO:
	}

	/**
	 * Remove a specific condition from the constraint store
	 * @param con The condition to be removed
	 */
	void ConstraintStore::removeCondition(Condition* con)
	{
		//TODO:
	}

	void ConstraintStore::acceptQuery(shared_ptr<ConstraintQuery> query)
	{
		//TODO:
	}

} /* namespace supplementary */
