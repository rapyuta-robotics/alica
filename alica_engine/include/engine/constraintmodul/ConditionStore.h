#pragma once

#include <engine/Types.h>

#include <map>
#include <memory>
#include <mutex>
#include <vector>

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
    void addCondition(const Condition* con);
    void removeCondition(const Condition* con);

    void acceptQuery(Query& query, std::shared_ptr<RunningPlan> rp) const;

    ConditionStore(const ConditionStore&) = delete;
    ConditionStore(ConditionStore&&) = delete;
    ConditionStore& operator=(const ConditionStore&) = delete;
    ConditionStore& operator=(ConditionStore&&) = delete;

  private:
    ConditionGrp _activeConditions;
    std::map<const Variable*, ConditionGrp> _activeVar2CondMap;

    mutable std::mutex _mtx;
};

} // namespace alica
