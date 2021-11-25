#pragma once
#include "engine/Types.h"

#include <unordered_set>

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

    void clear();
    EntryPointGrp succeededEntryPoints(std::size_t parentContextHash, const AbstractPlan* p) const;
    void removePlan(const AbstractPlan* plan);
    void markSuccessful(std::size_t parentContextHash, const EntryPoint* e);

    bool succeeded(const AbstractPlan* p, const EntryPoint* e) const;
    bool anyTaskSucceeded(const AbstractPlan* p) const;
    std::vector<std::size_t> toContextGrp() const;

private:
    std::unordered_set<std::size_t> _successMarks;
};

} /* namespace alica */
