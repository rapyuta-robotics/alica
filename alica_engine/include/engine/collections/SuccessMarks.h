#pragma once
#include "engine/Types.h"

#include <map>

namespace alica
{

class PlanRepository;

/**
 * Globally holds information about succeeded entry points for a specific robot
 */
class SuccessMarks
{
public:
    SuccessMarks();
    ~SuccessMarks();

    void limitToPlans(const AbstractPlanGrp& active);
    void update(const PlanRepository& planRepository, const IdGrp& succeededEps);
    const std::map<const AbstractPlan*, EntryPointGrp>& getSuccessMarks() const { return _successMarks; }

    void clear();
    const EntryPointGrp* succeededEntryPoints(const AbstractPlan* p) const;
    void removePlan(const AbstractPlan* plan);
    void markSuccessful(const AbstractPlan* p, const EntryPoint* e);

    bool succeeded(const AbstractPlan* p, const EntryPoint* e) const;
    bool anyTaskSucceeded(const AbstractPlan* p) const;
    IdGrp toIdGrp() const;

private:
    std::map<const AbstractPlan*, EntryPointGrp> _successMarks;
};

} /* namespace alica */
