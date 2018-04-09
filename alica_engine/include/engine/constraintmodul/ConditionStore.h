#pragma once

#include <map>
#include <list>
#include <vector>
#include <mutex>
#include <memory>

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
    void addCondition(const Condition* con);
    void removeCondition(const Condition* con);

    void acceptQuery(Query& query, std::shared_ptr<RunningPlan> rp) const;
    std::list<const Condition*> activeConditions;
    std::map<const Variable*, std::shared_ptr<std::vector<const Condition*>>> activeVar2CondMap;

    mutable std::mutex mtx;
};

}  // namespace alica
