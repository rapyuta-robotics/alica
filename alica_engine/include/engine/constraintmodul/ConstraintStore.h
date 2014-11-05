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
#include <vector>
#include <mutex>
#include <memory>
#include <algorithm>

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
		ConstraintStore();
		virtual ~ConstraintStore();
		void clear();
		void addCondition(Condition* con);
		void removeCondition(Condition* con);

		void acceptQuery(shared_ptr<ConstraintQuery> query, shared_ptr<RunningPlan> rp);
		list<Condition*> activeConditions;
		map<Variable*, shared_ptr<vector<Condition*>> > activeVariables;

		mutex mtx;
	};

} /* namespace supplementary */

#endif /* CONSTRAINTSTORE_H_ */
