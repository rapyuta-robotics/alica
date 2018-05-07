#pragma once

#include <algorithm>
#include <list>
#include <map>
#include <memory>
#include <unordered_set>

#include "engine/Types.h"

namespace alica
{

class AbstractPlan;
class EntryPoint;
class AlicaEngine;

/**
 * Globally holds information about succeeded entrypoints for a specific robot
 */
class SuccessMarks
{
  public:
    SuccessMarks(const AlicaEngine* ae);
    SuccessMarks(const AlicaEngine* ae, const std::list<int64_t>& epIds);
    virtual ~SuccessMarks();

    void limitToPlans(const AbstractPlanGrp& active);
    const std::map<const AbstractPlan*, std::shared_ptr<std::list<const EntryPoint*>>>& getSuccessMarks() const { return successMarks; }

    void clear();
    std::shared_ptr<std::list<const EntryPoint*>> succeededEntryPoints(const AbstractPlan* p) const;
    void removePlan(const AbstractPlan* plan);
    void markSuccessfull(const AbstractPlan* p, const EntryPoint* e);
    bool succeeded(const AbstractPlan* p, const EntryPoint* e) const;
    bool succeeded(int64_t planId, int64_t entryPointId) const;
    bool anyTaskSucceeded(const AbstractPlan* p) const;
    std::list<int64_t> toList() const;

  protected:
    std::map<const AbstractPlan*, std::shared_ptr<std::list<const EntryPoint*>>> successMarks;
    const AlicaEngine* ae;
};

} /* namespace alica */
