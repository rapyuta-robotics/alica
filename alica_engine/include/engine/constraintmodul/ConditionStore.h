#pragma once

#include <map>
#include <list>
#include <vector>
#include <mutex>
#include <memory>
#include <algorithm>

namespace alica
{
	class Variable;
	class Condition;
	class Query;
	class RunningPlan;

	using std::shared_ptr;
	using std::list;
	using std::map;
	using std::vector;
	using std::mutex;

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
