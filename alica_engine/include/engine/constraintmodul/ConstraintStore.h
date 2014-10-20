/*
 * ConstraintStore.h
 *
 *  Created on: Jul 15, 2014
 *      Author: Paul Panin
 */

#ifndef CONSTRAINTSTORE_H_
#define CONSTRAINTSTORE_H_

#include <map>
#include <list>
#include <unordered_set>

#include <memory>

using namespace std;

namespace alica
{
	class Variable;
	class Condition;
	class ConstraintQuery;
	class RunningPlan;

	/**
	 * Holds information about active constraints in the corresponding RunningPlan
	 */
	class ConstraintStore
	{
	public:
		ConstraintStore(RunningPlan* rp);
		virtual ~ConstraintStore();
		void clear();
		void addCondition(Condition* con);
		void removeCondition(Condition* con);

		//	TODO
		void acceptQuery(shared_ptr<ConstraintQuery> query);
		unordered_set<Condition*> activeConitions;
		map<Variable*, list<Condition*> > activeVariables;
		RunningPlan* rp;

	};

} /* namespace supplementary */

#endif /* CONSTRAINTSTORE_H_ */
