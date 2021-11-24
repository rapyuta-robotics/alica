#pragma once
#include "engine/Types.h"

#include <map>

namespace alica
{

class AlicaEngine;

/**
 * Globally holds information about succeeded entry points for a specific robot
 */
class SuccessMarks
{
public:
    SuccessMarks();
    ~SuccessMarks();

    void limitToPlans(const AbstractPlanGrp& active);
    void update(const AlicaEngine* ae, const IdGrp& succeededEps);
    const std::map<std::optional<std::size_t>, const EntryPoint*>& getSuccessMarks() const { return _successMarks; }

    void clear();
    const EntryPointGrp* succeededEntryPoints(const AbstractPlan* p) const;
    void removePlan(const AbstractPlan* plan);
    void markSuccessful(std::optional<std::size_t> parentRpContext, const EntryPoint* e);

    bool succeeded(const AbstractPlan* p, const EntryPoint* e) const;
    bool anyTaskSucceeded(const AbstractPlan* p) const;
    std::vector<std::size_t> toContextGrp() const;

private:
    std::map<std::optional<std::size_t>, const EntryPoint*> _successMarks;
};

} /* namespace alica */
