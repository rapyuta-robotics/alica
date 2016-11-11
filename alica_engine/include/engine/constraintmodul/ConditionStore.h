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
	class Query;
	class RunningPlan;

	/**
	 * Holds information about active constraints in the corresponding RunningPlan
	 */
	class ConditionStore
	{
	public:
		ConditionStore();
		virtual ~ConditionStore();
		void clear();
		void addCondition(Condition* con);
		void removeCondition(Condition* con);

		void acceptQuery(shared_ptr<Query> query, shared_ptr<RunningPlan> rp);
		list<Condition*> activeConditions;
		map<Variable*, shared_ptr<vector<Condition*>> > activeVar2CondMap;

		mutex mtx;
	};

} /* namespace supplementary */

#endif /* CONSTRAINTSTORE_H_ */
