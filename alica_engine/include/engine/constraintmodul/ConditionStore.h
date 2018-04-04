#pragma once

#include <map>
#include <list>
#include <vector>
#include <mutex>
#include <memory>
#include <algorithm>

namespace alica {
class Variable;
class Condition;
class Query;
class RunningPlan;

/**
 * Holds information about active constraints in the corresponding RunningPlan
 */
class ConditionStore {
public:
    ConditionStore();
    virtual ~ConditionStore();
    void clear();
    void addCondition(Condition* con);
    void removeCondition(Condition* con);

    void acceptQuery(Query& query, std::shared_ptr<RunningPlan> rp) const;
    std::list<Condition*> activeConditions;
    std::map<Variable*, std::shared_ptr<std::vector<Condition*>>> activeVar2CondMap;

    mutable std::mutex mtx;
};

}  // namespace alica
